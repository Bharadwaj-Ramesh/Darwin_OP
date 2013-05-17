#include "gazebo.hh"
#include "plugins/CameraPlugin.hh"
#include "gazebo/common/common.hh"
#include "gazebo/transport/transport.hh"
#include <gazebo/msgs/msgs.hh>
#include <math/gzmath.hh>
#include <iostream>
#include <fstream>
using namespace std;
namespace gazebo

    {   
      class CameraDump : public CameraPlugin
      { 
        public: CameraDump() : CameraPlugin(),saveCount(0) {}

        public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
        {
          // Don't forget to load the camera plugin
          CameraPlugin::Load(_parent,_sdf);
//          myfile1.open("Cb.csv");
          
        } 

        // Update the controller
        public: void OnNewFrame(const unsigned char *_image, 
            unsigned int _width, unsigned int _height, unsigned int _depth, 
            const std::string &_format)
        {
          if (true)
          {
          char tmp[1024];
//          snprintf(tmp, sizeof(tmp), "/tmp/%s-%04d.jpg",
//              this->parentSensor->GetCamera()->GetName().c_str(), this->saveCount);
          unsigned int imgW;
          unsigned int imgH;
          imgW = this->parentSensor->GetCamera()->GetImageWidth ();
          imgH = this->parentSensor->GetCamera()->GetImageHeight ();
          gzdbg << "imgW : " << imgW << "]\n";
          gzdbg << "imgH : " << imgH << "]\n";
          const unsigned char* curImg;
          double imData;
          double theImg[imgH][imgW][3];
          double y1;
          double y2;
          double y[imgH][imgW]; // the intensity value
          double Cb[imgH][imgW];
          double Cr[imgH][imgW];
          area  = 0;
          xcord = 0;
          ycord = 0;
          curImg = this->parentSensor->GetCamera()->GetImageData ();
          std::cout << "I am running\n";
//          this->parentSensor->GetCamera()->SaveFrame(
//                _image, _width, _height, _depth, _format, tmp);

         
          for (int i = 0; i < imgH ; i++){
            for (int j = 0; j < imgW ; j++){
              for (int k = 0; k < 3 ; k++){
              imData = *curImg;
//              gzdbg << "imData : " << imData << "]\n";         
              theImg[i][j][k] = imData;
              curImg++;
              }
            }
          }

//converting to ycbcr
// this loop calculates the Cb and Cr value 
          for (int i = 0; i < imgH ; i++){
            for (int j = 0; j < imgW ; j++){
              y1 = (.299*theImg[i][j][0] ) + (0.587 *theImg[i][j][1]) + (0.114 *theImg[i][j][2]);
              y2 = ((theImg[i][j][0] - y1)*.713 + 128); // This is the Cr value
              if (y2 > 180)
                Cr[i][j] = 1; // the Cr value
              else
                Cr[i][j] = 0; // the Cr value                 
            }
          }


          // This is to test is I have the right 
          for (int i = 0; i < imgH ; i++){
            for (int j = 0; j < imgW ; j++){
               if (Cr[i][j] == 1)
                 {xcord = xcord + j;
                  ycord = ycord + i;}
                  area = area + Cr[i][j];
             }
          }
          xcord = xcord/area;
          ycord = ycord/area;
          dist  = 4.667168 * pow(area,-0.41198);
          std::cout << "area : " << area << "\n";
          std::cout << "Distance : " << dist << "\n";
          std::cout << "(x,y) = (" << xcord << "," << ycord << ")\n";
          if (area!=0)
             this->saveCount++;
        }

         if (this->saveCount == 10 )
         {
         gazebo::transport::NodePtr node(new gazebo::transport::Node());
         node->Init();

         // Start transport
         gazebo::transport::run();

         // Publish to a Gazebo topic
         gazebo::transport::PublisherPtr pub =
         node->Advertise<gazebo::msgs::Vector3d>("~/pose_example1");

         // Wait for a subscriber to connect
         pub->WaitForConnection();

         // Publisher loop...replace with your own code.
         while (true)
         {
           gazebo::common::Time::MSleep(100);
    	   gazebo::math::Vector3 vect(xcord, ycord,dist);
	   gazebo::msgs::Vector3d msg;
	   gazebo::msgs::Set(&msg, vect);
	   pub->Publish(msg);
         }

         // Make sure to shut everything down.
         gazebo::transport::fini(); 
        }  
        }
        private: int saveCount;
        double area ;
        double dist;
        double xcord ;
        double ycord ;

};

      // Register this plugin with the simulator
      GZ_REGISTER_SENSOR_PLUGIN(CameraDump)
    }

