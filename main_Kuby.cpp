#include <iostream>
#include <vector>
#include <thread>
#include <cmath>
#include <chrono>

#include "Position.hpp"
#include "Laser.hpp"
#include "WheeledRobot.hpp"
#include "PoseChange.hpp"
#include "algorithms.hpp"

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

#include <opencv2/opencv.hpp>
#include <boost/asio.hpp>

using namespace sl;

static const int MAP_SIZE_PIXELS        = 800;
static const double MAP_SIZE_METERS     =  12;

static const int SCAN_SIZE 		        = 545;

#define MAXLINE 10000

int coords2index(double x,  double y)
{    
    return y * MAP_SIZE_PIXELS + x;
}

int mm2pix(double mm)
{
    return (int)(mm / (MAP_SIZE_METERS * 1000. / MAP_SIZE_PIXELS));  
}

class MinesRPLidarA1 : public Laser
{
    public:
        
        MinesRPLidarA1(int detection_margin = 0, float offset_mm = 0) :
        Laser(SCAN_SIZE, 5.0f, 360, 1200, detection_margin, offset_mm)
        {
        }
        
        MinesRPLidarA1(void) : Laser() {}
    };

class Rover : WheeledRobot
    {
        
    public:
        
        Rover() : WheeledRobot(
            77,     // wheelRadiusMillimeters
            165)     // halfAxleLengthMillimeters
        {
        }
        
        PoseChange computePoseChange(
                double timestamp,
                double left_wheel_odometry,
                double right_wheel_odometry)
        {  
            return WheeledRobot::computePoseChange(
                    timestamp,
                    left_wheel_odometry,
                    right_wheel_odometry);
        }

    protected:    
        
        void extractOdometry(
            double timestamp, 
            double leftWheelOdometry, 
            double rightWheelOdometry, 
            double & timestampSeconds, 
            double & leftWheelDegrees, 
            double & rightWheelDegrees)
        {        
            // Convert microseconds to seconds, ticks to angles        
            timestampSeconds = timestamp / 1e6;
            leftWheelDegrees = ticksToDegrees(leftWheelOdometry);
            rightWheelDegrees = ticksToDegrees(rightWheelOdometry);
        }
        
        void descriptorString(char * str)
        {
            sprintf(str, "ticks_per_cycle=%d", this->TICKS_PER_CYCLE);
        }
            
    private:
        
        double ticksToDegrees(double ticks)
        {
            return ticks * (180. / this->TICKS_PER_CYCLE);
        }
        
        static const int TICKS_PER_CYCLE = 2000;
};
//komunikacja
    using boost::asio::ip::tcp;
    const uint32_t data_size = 800 * 800;
    std::atomic<bool> client_connected(false);
    std::unique_ptr<tcp::socket> socket_ptr;
    std::atomic<bool> server_running(true);
    std::string connect_status;


    void sendFrame(tcp::socket& socket, const unsigned char* frame){
        try{
            int32_t size = htonl(data_size);
            boost::asio::write(socket, boost::asio::buffer(&size, sizeof(size)));
            boost::asio::write(socket, boost::asio::buffer(frame, data_size));
        }
        catch(std::exception& e){
            socket_ptr.reset(); // Zamknij i usuń gniazdo
            client_connected = false;
            std::cerr<< "\nBłąd wysyłania ramki: "<<e.what()<<std::endl<<std::endl;
            connect_status = "Brak połączenia";
        }
    }

int main() {
    std::string const port = "/dev/ttyUSB0";
    int const baudrate = 115200;
    int const NODE_SIZE = SCAN_SIZE;
    std::chrono::duration<double> duration;
    double freq;

    bool running = true;

    sl_result result;

    auto channelResult = createSerialPortChannel(port, baudrate);
    if (channelResult) {
        std::cout << "Stworzono kanal komunikacyjny." << std::endl;
    } else {
        std::cerr << "Nie udało się utworzyć kanalu komunikacyjnego." << std::endl;
        return -1;
    };
    
    IChannel* channel = *channelResult;

    ILidarDriver* lidarDriver = *createLidarDriver();
    if (lidarDriver) {
        std::cout << "Stworzono sterownik LIDAR" << std::endl;
    } else {
        std::cerr << "Nie udalo sie utworzyc sterownika LIDAR." << std::endl;
        return -1;
    };

    result = lidarDriver->connect(channel);

    if (SL_IS_OK(result)) {
        std::cout << "Polaczono sie z LIDAR." << std::endl;
    } else {
        std::cerr << "Nie udalo sie polaczyc z LIDAR." << std::endl;
        return -1;
    };
    //Inicjalizacja SLAM
    bool use_odometry = 0;
    int random_seed = 9999;

    vector<int *> scans;
    vector<long *> odometries;

    //Old code
    Rover robot = Rover();

    unsigned char * mapbytes = new unsigned char[MAP_SIZE_PIXELS * MAP_SIZE_PIXELS];

    MinesRPLidarA1 laser(0,0);
    SinglePositionSLAM * slam = random_seed ?
    (SinglePositionSLAM*)new RMHC_SLAM(laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS, random_seed) :
    (SinglePositionSLAM*)new Deterministic_SLAM(laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS);


    vector<double *> trajectory;

    lidarDriver->setMotorSpeed();

    if (SL_IS_OK(result)) {
        std::cout << "Uruchomiono silnik." << std::endl;
    } else {
        std::cerr << "Nie udalo się uruchomic silnika." << std::endl;
        return -1;
    };

    lidarDriver->startScan(0, 1);

    std::thread ExitLoopThread([&running]() {
        std::cin.get();       
        running = false;
    });

    boost::asio::io_context io_context;
    tcp::acceptor acceptor(io_context, tcp::endpoint(tcp::v4(), 12345));
    tcp::socket socket(io_context);
    
    std::thread server_thread([&io_context, &acceptor, &socket_ptr, &client_connected, mapbytes]() {
        while (server_running) {
            try {
                socket_ptr = std::make_unique<tcp::socket>(io_context);
                acceptor.accept(*socket_ptr);
                client_connected = true;
                connect_status = "Połączono";

                // Oczekuj na rozłączenie
                while (client_connected) {
                    if (socket_ptr->available() == 0) {
                        // Sprawdzanie czy klient wciąż jest podłączony
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                    if (client_connected && socket_ptr) {
                        try {
                            sendFrame(*socket_ptr, mapbytes);
                        } catch (const std::exception& e) {
                            socket_ptr.reset();
                            client_connected = false;
                            std::cerr<<"\nBłąd połączenia: "<< e.what() << std::endl << std::endl;
                            connect_status = "Brak połączenia"; 
                        }
                        } else {
                            connect_status = "Brak połączenia";
                        }
                    
                }
            } catch (const std::exception& e) {
                std::cerr << "Błąd serwera lub połączenia: " << e.what() << std::endl;
                client_connected = false;
                socket_ptr.reset();
            }
        }
    });
    while (running) {
        auto time = std::chrono::high_resolution_clock::now();
        sl_lidar_response_measurement_node_hq_t nodes[NODE_SIZE];
        size_t count = sizeof(nodes) / sizeof(nodes[0]);
    
        result = lidarDriver->grabScanDataHq(nodes, count);

        //odebranie odometrii


        //
        for (auto scan : scans) {
            delete[] scan;
        }
        scans.clear();
            if (SL_IS_OK(result)) {
                SinglePositionSLAM * singleSlam = dynamic_cast<SinglePositionSLAM*>(slam);
            if (singleSlam) {
                singleSlam->setLaserScanSize(static_cast<int>(count));
            }
        laser.setScanSize(count);
            lidarDriver->ascendScanData(nodes, count);
            int *scanvals = new int[count];
            for (int i = 0; i < (int)count; i++) {
                double dist = nodes[i].dist_mm_q2 / 4.0f;

                scanvals[i] = static_cast<int>(std::round(dist));
            }

            /* srednia z pobliskich skanow:
            for (int i = 1; i < (int)count - 1; i++) {
                if(scanvals[i] == 0)
                {
                    scanvals[i] = (scanvals[i-1] + scanvals[i+1]) / 2;
                }
            }   
            */        

            scans.push_back(scanvals);

            int * lidar = scans[0];
            if (use_odometry)
            {
                long * o = odometries[0];
                PoseChange poseChange = robot.computePoseChange(o[0], o[1], o[2]);
                slam->update(lidar, poseChange);            
            }
            else
            {
                slam->update(lidar);  
            }
            Position position = slam->getpos();
            double * v = new double[2];
            v[0] = position.x_mm;
            v[1] = position.y_mm;
            trajectory.push_back(v);
            slam->getmap(mapbytes);

            for (int k=0; k<(int)trajectory.size(); ++k)
            {        
                double * v = trajectory[k];
                            
                int x = mm2pix(v[0]);
                int y = mm2pix(v[1]);
                                
                mapbytes[coords2index(x, y)] = 0;
            }
            cv::Mat mapImage(MAP_SIZE_PIXELS, MAP_SIZE_PIXELS, CV_8UC1, mapbytes);

            cv::imshow("Map", mapImage);
            cv::waitKey(1);
            //wyslanie obrazu do aplikacji
            duration = std::chrono::high_resolution_clock::now() - time;
            freq = (duration.count() > 0) ? 1.0 / duration.count() : 0.0;
            printf("Liczba skanów: %d  Częstotliwość: %f  Połączenie: %s\n",laser.getScanSize(),freq,connect_status.c_str());
        }
    }
    lidarDriver->stop();
    lidarDriver->setMotorSpeed(0);

    delete lidarDriver;

    for (auto scan : scans) {
        delete[] scan;
    }

    delete[] mapbytes;
    cv::destroyAllWindows();
    ExitLoopThread.join();
    server_running = false;
    client_connected = false;
    socket_ptr.reset();
    acceptor.close();
    acceptor.cancel();
    io_context.stop();
    server_thread.join();
    return 0;
}