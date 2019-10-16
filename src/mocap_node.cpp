/// \author <a href="mailto:graeve@ais.uni-bonn.de">Kathrin Gräve</a>
///
/// ROS node that translates motion capture data from an OptiTrack rig to tf transforms.
/// The node receives the binary packages that are streamed by the Arena software,
/// decodes them and broadcasts the poses of rigid bodies as tf transforms.
///
/// Currently, this node supports the NatNet streaming protocol v1.4.

// Local includes
#include "mocap_optitrack/socket.h"
#include "mocap_optitrack/mocap_datapackets.h"
#include "mocap_optitrack/mocap_config.h"
#include "mocap_optitrack/skeletons.h"

// ROS includes
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>

// System includes
#include <string>
#include <tuple>
#include <unistd.h>
#include <math.h>
////////////////////////////////////////////////////////////////////////
// Constants

// ip on multicast group - cannot be changed in Arena
const std::string MULTICAST_IP_KEY = "optitrack_config/multicast_address";
const std::string MULTICAST_IP_DEFAULT = "224.0.0.1";

const std::string MOCAP_MODEL_KEY = "mocap_model";
const std::string RIGID_BODIES_KEY = "rigid_bodies";
const std::string MARKER_KEY = "markers";
const char ** DEFAULT_MOCAP_MODEL = OBJECT;
//const char ** DEFAULT_MOCAP_MODEL = SKELETON_WITHOUT_TOES;

const int COMMAND_PORT = 1510;
const int LOCAL_PORT = 1511;

// NATNET message ids
#define NAT_PING                    0
#define NAT_PINGRESPONSE            1
#define NAT_REQUEST                 2
#define NAT_RESPONSE                3
#define NAT_REQUEST_MODELDEF        4
#define NAT_MODELDEF                5
#define NAT_REQUEST_FRAMEOFDATA     6
#define NAT_FRAMEOFDATA             7
#define NAT_MESSAGESTRING           8
#define NAT_UNRECOGNIZED_REQUEST    100
#define UNDEFINED                   999999.9999
#define MAX_PACKETSIZE              100000  // max size of packet (actual packet size is dynamic)
#define MAX_NAMELENGTH              256

// sender
typedef struct
{
    char szName[MAX_NAMELENGTH];            // sending app's name
    unsigned char Version[4];               // sending app's version [major.minor.build.revision]
    unsigned char NatNetVersion[4];         // sending app's NatNet version [major.minor.build.revision]

} sSender;

typedef struct
{
    unsigned short iMessage;                // message ID (e.g. NAT_FRAMEOFDATA)
    unsigned short nDataBytes;              // Num bytes in payload
    union
    {
        unsigned char  cData[MAX_PACKETSIZE];
        char           szData[MAX_PACKETSIZE];
        unsigned long  lData[MAX_PACKETSIZE/4];
        float          fData[MAX_PACKETSIZE/4];
        sSender        Sender;
    } Data;                                 // Payload

} sPacket;

////////////////////////////////////////////////////////////////////////

double square_distance_fn(Marker& a, Marker& b)
{
  double disX = a.x - b.x, disY = a.y - b.y, disZ = a.z - b.z, dis = 0.0;
  if( abs(disX) > 0.0001)
    dis += disX * disX;
  if( abs(disY) > 0.0001)
    dis += disY * disY;
  if( abs(disZ) > 0.0001)
    dis += disZ * disZ;
  return dis;
}

void trackMarkers(int num_mocap_markers, Marker* mocap_markers, MarkerArray& published_unlabeled_markers)
{
  int n = published_unlabeled_markers.size(), m = num_mocap_markers;
  std::vector<std::tuple<double, int, int>> square_distance;

  for(int i = 0; i < n; ++i)
    for(int j = 0; j < m; ++j)
      square_distance.push_back(std::make_tuple(
        square_distance_fn(published_unlabeled_markers[i].currentMarker, mocap_markers[j]),
        i, j));

  sort(square_distance.begin(), square_distance.end());

  std:;vector<bool> unassigned(n, true);
  std::vector<bool> unused(m, true);

  for(int _id = 0; _id < square_distance.size(); ++_id)
    if(unassigned[std::get<1>(square_distance[_id])] &&
       unused[std::get<2>(square_distance[_id])])
    {
      double & square_dis = std::get<0>(square_distance[_id]);
      int i = std::get<1>(square_distance[_id]),
          j = std::get<2>(square_distance[_id]);
      PublishedMarker& marker = published_unlabeled_markers[i];
      if(square_dis < 0.002 || square_dis < 0.002 + marker.disconnectedFrames * 0.00002)
      {
        marker.update(mocap_markers[j]);
        if(marker.disconnectedFrames > 0)
        {
          marker.disconnectedFrames = 0;
          ROS_INFO("Recover lost marker %d, distance from last known location %.4f",
                   i, sqrt(square_dis));
        }
        unassigned[i] = false;
        unused[j] = false;
      }
    }

  for(int i = 0; i < n; ++i) {
    if(unassigned[i])
    {
      published_unlabeled_markers[i].disconnectedFrames += 1;
      ROS_INFO("Lost unlabeled marker %d for %d frames", //, which was at %.4f, %.4f, %.4f",
                i, published_unlabeled_markers[i].disconnectedFrames);
                //currentMarker.x, currentMarker.y, currentMarker.z);
    }
  }


  //ROS_DEBUG("NumOtherMarkers: %d", numOtherMarkers);
  //for( int i = 0; i < numOtherMarkers; ++i)
  //{
  //  ROS_DEBUG("X %f Y %f Z %f", format.model.otherMarkers[i].x, format.model.otherMarkers[i].y, format.model.otherMarkers[i].z);
  //}
}

////////////////////////////////////////////////////////////////////////

void processMocapData( const char** mocap_model,
                       RigidBodyMap& published_rigid_bodies,
                       PublishedPointArray& published_pArray,
                       const std::string& multicast_ip)
{
  MarkerArray& published_unlabeled_markers = published_pArray.published_unlabeled_markers;

  UdpMulticastSocket multicast_client_socket( LOCAL_PORT, multicast_ip );

  unsigned short payload_len;
  int numberOfPackets = 0;
  int nver[4] = {0,0,0,0}; // natnet version
  int sver[4] = {0,0,0,0}; // server version

  sPacket PacketOut;
  PacketOut.iMessage = NAT_PING;
  PacketOut.nDataBytes = 0;
  int nTries = 3;
  while(nTries--) {
  }

  ROS_INFO("Start processMocapData");
  bool version = false;

  while(ros::ok())
  {
    bool packetread = false;
    int numBytes = 0;
    sPacket PacketIn;

    if(!version) {
      int iRet = multicast_client_socket.send((char*)&PacketOut, 4 + PacketOut.nDataBytes, COMMAND_PORT);
    }

    do
    {
      // Receive data from mocap device
      numBytes = multicast_client_socket.recv();

      // Parse mocap data
      if( numBytes > 0 )
      {
        const char* buffer = multicast_client_socket.getBuffer();
        memcpy((char*)&PacketIn, buffer, numBytes);
        unsigned short header = *((unsigned short*)(&buffer[0])); // 2-bytes, ushort.


        // Look for the beginning of a NatNet package
        if (header == NAT_FRAMEOFDATA && version)
        {
          payload_len = *((unsigned short*) &buffer[2]);  // 2-bytes.
          MoCapDataFormat format(buffer, payload_len);
          format.setVersion(nver,sver);
          format.parse();
          packetread = true;
          numberOfPackets++;

          // Update rigid body information
          if( format.model.numRigidBodies > 0 && published_rigid_bodies.size() > 0)
          {
            for( int i = 0; i < format.model.numRigidBodies; i++ )
            {
              int ID = format.model.rigidBodies[i].ID;
              RigidBodyMap::iterator item = published_rigid_bodies.find(ID);

              if (item != published_rigid_bodies.end())
              {
                item->second.updateMarker(format.model.rigidBodies[i].marker, format.model.rigidBodies[i].NumberOfMarkers);
                item->second.publish(format.model.rigidBodies[i]);
              }
            }
          }

          // Process other marker information
          if( format.model.numOtherMarkers > 0 &&
              published_unlabeled_markers.size() > 0)
          {
            trackMarkers(format.model.numOtherMarkers, format.model.otherMarkers, published_unlabeled_markers);

            for (int i = 0; i < published_unlabeled_markers.size(); ++i)
              published_unlabeled_markers[i].publish();
          }

          published_pArray.publish();
        }

        if (header == NAT_PINGRESPONSE) {
          ROS_DEBUG("Header : %d, %d", header, PacketIn.iMessage);
          ROS_DEBUG("nData : %d", PacketIn.nDataBytes);

          for(int i=0;i<4;++i) {
            nver[i] = (int)PacketIn.Data.Sender.NatNetVersion[i];
            sver[i] = (int)PacketIn.Data.Sender.Version[i];
          }

          ROS_INFO_ONCE("NATNet Version : %d.%d.%d.%d", nver[0], nver[1], nver[2], nver[3]);
          ROS_INFO_ONCE("Server Version : %d.%d.%d.%d", sver[0], sver[1], sver[2], sver[3]);
          version = true;
        }
        // else skip packet
      }
    } while( numBytes > 0 );

    // Don't try again immediately
    if( !packetread )
    {
      usleep( 10 );
    }
  }
}



////////////////////////////////////////////////////////////////////////

int main( int argc, char* argv[] )
{
  // Initialize ROS node
  ros::init(argc, argv, "mocap_node");
  ros::NodeHandle n("~");

  // Get configuration from ROS parameter server
  const char** mocap_model( DEFAULT_MOCAP_MODEL );
  if( n.hasParam( MOCAP_MODEL_KEY ) )
  {    std::string tmp;
    if( n.getParam( MOCAP_MODEL_KEY, tmp ) )
    {
      if( tmp == "SKELETON_WITH_TOES" )
        mocap_model = SKELETON_WITH_TOES;
      else if( tmp == "SKELETON_WITHOUT_TOES" )
        mocap_model = SKELETON_WITHOUT_TOES;
      else if( tmp == "OBJECT" )
        mocap_model = OBJECT;
    }
  }

  // Get configuration from ROS parameter server
  std::string multicast_ip( MULTICAST_IP_DEFAULT );
  if( n.hasParam( MULTICAST_IP_KEY ) )
  {
    n.getParam( MULTICAST_IP_KEY, multicast_ip );
  }
  else {
    ROS_WARN_STREAM("Could not get multicast address, using default: " << multicast_ip);
  }

  // pArray stores all marker information (markers that belong to rigid body and unlabeled markers)
  PublishedPointArray published_pArray(n);

  // Add each rigid body in config to the RigidBodyMap
  RigidBodyMap published_rigid_bodies;
  if (n.hasParam(RIGID_BODIES_KEY))
  {
      XmlRpc::XmlRpcValue body_list;
      n.getParam("rigid_bodies", body_list);
      if (body_list.getType() == XmlRpc::XmlRpcValue::TypeStruct && body_list.size() > 0)
      {
          XmlRpc::XmlRpcValue::iterator i;
          for (i = body_list.begin(); i != body_list.end(); ++i) {
              if (i->second.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                  PublishedRigidBody body(i->second, published_pArray.published_model_markers);
                  string id = (string&) (i->first);
                  RigidBodyItem item(atoi(id.c_str()), body);

                  std::pair<RigidBodyMap::iterator, bool> result = published_rigid_bodies.insert(item);
                  if (!result.second)
                  {
                      ROS_ERROR("Could not insert configuration for rigid body ID %s", id.c_str());
                  }
              }
          }
      }
  }

  // Add each marker in config to the pArray
  if (n.hasParam(MARKER_KEY))
  {
      XmlRpc::XmlRpcValue marker_list;
      n.getParam(MARKER_KEY, marker_list);
      if (marker_list.getType() == XmlRpc::XmlRpcValue::TypeStruct && marker_list.size() > 0)
      {
          XmlRpc::XmlRpcValue::iterator i;
          for (i = marker_list.begin(); i != marker_list.end(); ++i) {
              if (i->second.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                  PublishedMarker marker(i->second);
                  published_pArray.published_unlabeled_markers.push_back(marker);
                  ROS_INFO("Tracking marker ID %s", i->first.c_str());
              }
          }
      }
      ROS_INFO("Tracking %d unlabeled markers in total", int(published_pArray.published_unlabeled_markers.size()));

  }

  // Process mocap data until SIGINT
  processMocapData(mocap_model, published_rigid_bodies, published_pArray, multicast_ip);

  return 0;
}
