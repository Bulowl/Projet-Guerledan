#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ros/ros.h>
#include <nmea_msgs/Sentence.h>
#include <zodiac_command/WaypointListMission.h>

using namespace std;

float nmeaToDeg(float nmeaNro)
{
  return floor(nmeaNro/100.) + (nmeaNro/100.-floor(nmeaNro/100.))*100/60;
}

float degToNmea(float deg)
{
  return floor(deg)*100. + (deg - floor(deg))*60;
}

struct NMEA_WPL
{
  NMEA_WPL(string datastring)
  {
    std::stringstream parts(datastring);
    string part;

    std::getline(parts,part,',');

    std::getline(parts,part,',');
    this->latitude = nmeaToDeg(std::stod(part));
    std::getline(parts,part,',');
    this->latitudeH = part;
    if(this->latitudeH.compare("S") == 0)
      this->latitude = -this->latitude;

    std::getline(parts,part,',');
    this->longitude = nmeaToDeg(std::stod(part));
    std::getline(parts,part,',');
    this->longitudeH = part;
    if(this->longitudeH.compare("W") == 0)
      this->longitude = -this->longitude;

    this->name = part;
  }

  string toString()
  {
    std::stringstream ss;
    ss << this->name << ": " << this->latitude << ", " << this->longitude;
    return ss.str();
  }

  double latitude, longitude;
  string latitudeH, longitudeH, name;
};

struct NMEA_RTE
{
  NMEA_RTE(string datastring)
  {
    std::stringstream parts(datastring);
    string part;

    std::getline(parts,part,',');

    std::getline(parts,part,',');
    this->nro_total = std::stod(part);
    std::getline(parts,part,',');
    this->nro = std::stod(part);

    std::getline(parts,part,',');
    this->type = part;
    std::getline(parts,part,',');
    this->name = part;

    std::getline(parts,part,',');
    this->wps = part; //should improve this part
  }

  string toString()
  {
    std::stringstream ss;
    ss << this->nro << "/" << this->nro_total << " " <<this->name << ": " << this->wps;
    return ss.str();
  }

  double nro, nro_total;
  string type, name, wps;
};


class ZodiacAutonomous
{
public:

  ZodiacAutonomous () : loaded(true)
  {
    ros::NodeHandle nodeLocal("~");

    std::string ns = ros::this_node::getNamespace();

    sub1 = n.subscribe("nmea_sentence_opencpn", 1000, &ZodiacAutonomous::nmeaSentenceCallback, this);
    pub2 = n.advertise<zodiac_command::WaypointListMission>("new_waypoint_mission",1000,true);
  }

  void nmeaSentenceCallback(const nmea_msgs::Sentence sentence_msg)
  {
    // Test if it was sending twice
    if (sentence_msg.sentence != previousSentence)
    {
      cout << "NMEA received : " << sentence_msg.sentence << endl;

      std::stringstream parts(sentence_msg.sentence);
      string part;

      std::getline(parts,part,',');

      if(part.compare("$ECWPL") == 0)
      {
        if(loaded)
        {
          loaded = false;
          pts.clear();
        }
        NMEA_WPL wp(sentence_msg.sentence);
        pts.push_back(wp);
      }
      else if(part.compare("$ECRTE") == 0)
      {
        if(!loaded)
          publishWaypointList();
        loaded = true;
        NMEA_RTE rtem(sentence_msg.sentence);
        rte.push_back(rtem);
      }
    }
    else if(sentence_msg.sentence != ""){
      ROS_WARN("NMEA waypoint message received twice");
    }

    previousSentence = sentence_msg.sentence;
  }

  void publishWaypointList()
  {
    zodiac_command::WaypointListMission waypoint_msg;
    waypoint_msg.header.stamp = ros::Time::now();
    waypoint_msg.header.frame_id = "mission_waypoints";
    waypoint_msg.child_frame_id = "map";

    int i = 1;
    for(auto pt : pts)
    {
      zodiac_command::WaypointMission wp;
      wp.waypointID = i++;
      wp.latitude  = pt.latitude;
      wp.longitude = pt.longitude;
      wp.waypointReached = 0;
      waypoint_msg.waypoints.push_back(wp);
    }
    pub2.publish(waypoint_msg);
  }

private:
  ros::NodeHandle n;
  ros::Subscriber sub1;
  ros::Publisher pub2;
  vector<NMEA_WPL> pts;
  vector<NMEA_RTE> rte;
  bool loaded;
  string previousSentence; // in order to check if the NMEA sentence was recived twice.
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypointListInterface");

  ZodiacAutonomous mZodiacAutonomous;

  ros::spin();

  return 0;
}
