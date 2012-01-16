#include "mocap_datapackets.h"

#include <stdio.h>
#include <string>
#include <iostream>
using namespace std;


Marker::Marker()
{
}

Marker::~Marker()
{
}


RigidBody::RigidBody() 
  : pose(), marker(0)
{
}

RigidBody::~RigidBody()
{
  delete[] marker;
}

ModelDescription::ModelDescription()
  : markerNames(0)
{
}

ModelDescription::~ModelDescription()
{
  delete[] markerNames;
}

ModelFrame::ModelFrame()
  : markers(0), otherMarkers(0), rigidBodies(0)
{
}

ModelFrame::~ModelFrame()
{
  delete[] markers;
  delete[] otherMarkers;
  delete[] rigidBodies;
}

MoCapDataFormat::MoCapDataFormat(const char *packet, unsigned short length) 
  : packet(packet), length(length)
{
}

MoCapDataFormat::~MoCapDataFormat()
{
  delete[] model;
}

void MoCapDataFormat::seek(size_t count)
{
  packet += count;
  length -= count;
}

void MoCapDataFormat::parse()
{
  seek(4);
  
  // parse frame number
  frameNumber = *((int*) packet);
  seek(sizeof(int));

  // count number of packetsets
  numModels = *((int*) packet);
  seek(sizeof(int));

  model = new ModelFrame[numModels];
  for (int i = 0; i < numModels; i++)
  {
    while (*packet != '\0')
    {
      model[i].name.push_back(*packet);
      seek(1);
    }
    seek(1);

    // read number of markers that belong to the model
    model[i].numMarkers = *((int*) packet);
    seek(sizeof(int));

    model[i].markers = new Marker[model[i].numMarkers];
    for (int k = 0; k < model[i].numMarkers; k++)
    {
      // read marker positions
      model[i].markers[k] = *((Marker*) packet);
      seek(sizeof(Marker));
    }
 
    // read number of 'other' markers (cf. NatNet specs)
    model[i].numOtherMarkers = *((int*) packet);
    seek(sizeof(int));
    model[i].otherMarkers = new Marker[model[i].numOtherMarkers];
    for (int l = 0; l < model[i].numOtherMarkers; l++)
    {
      // read positions of 'other' markers
      model[i].otherMarkers[l] = *((Marker*) packet);
      seek(sizeof(Marker));
    }
 
    // read number of rigid bodies of the model
    model[i].numRigidBodies = *((int*) packet);
    seek(sizeof(int));
    model[i].rigidBodies = new RigidBody[model[i].numRigidBodies];
    for (int m = 0; m < model[i].numRigidBodies; m++)
    {
      // read id, position and orientation of each rigid body
      model[i].rigidBodies[m].ID =  *((int*) packet);
      seek(sizeof(int));
      model[i].rigidBodies[m].pose.position.x =  *((float*) packet);
      seek(sizeof(float));
      model[i].rigidBodies[m].pose.position.y =  *((float*) packet);
      seek(sizeof(float));
      model[i].rigidBodies[m].pose.position.z =  *((float*) packet);
      seek(sizeof(float));
      model[i].rigidBodies[m].pose.orientation.x =  *((float*) packet);
      seek(sizeof(float));
      model[i].rigidBodies[m].pose.orientation.y =  *((float*) packet);
      seek(sizeof(float));
      model[i].rigidBodies[m].pose.orientation.z =  *((float*) packet);
      seek(sizeof(float));
      model[i].rigidBodies[m].pose.orientation.w =  *((float*) packet);
      seek(sizeof(float));

      // get number of markers per rigid body
      model[i].rigidBodies[m].NumberOfMarkers =  *((int*) packet);
      seek(sizeof(int));
      model[i].rigidBodies[m].marker = new Marker [model[i].rigidBodies[m].NumberOfMarkers];
      for (int n = 0; n < model[i].rigidBodies[m].NumberOfMarkers; n++)
      {
        // get position for each marker
        model[i].rigidBodies[m].marker[n] = *((Marker*) packet);
        seek(sizeof(Marker));
      }

    }
    // get latency
    model[i].latency = *((float*) packet);
    seek(sizeof(float));
  }

}

