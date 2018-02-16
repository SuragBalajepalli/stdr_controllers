//path_client:
// illustrates how to send a request to the path_service service

#include <ros/ros.h>
#include <example_ros_service/PathSrv.h> // this message type is defined in the current package
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;
typedef vector <double> record_t;
typedef vector <record_t> data_t;

string g_ros_ws_path; // global string object

// see: http://www.cplusplus.com/forum/general/17771/
//-----------------------------------------------------------------------------
// Let's overload the stream input operator to read a list of CSV fields (which a CSV record).
// Remember, a record is a list of doubles separated by commas ','.

istream& operator >>(istream& ins, record_t& record) {
    // make sure that the returned record contains only the stuff we read now
    record.clear();

    // read the entire line into a string (a CSV record is terminated by a newline)
    string line;
    getline(ins, line);

    // now we'll use a stringstream to separate the fields out of the line
    stringstream ss(line);
    string field;
    while (getline(ss, field, ',')) {
        // for each field we wish to convert it to a double
        // (since we require that the CSV contains nothing but floating-point values)
        stringstream fs(field);
        double f = 0.0; // (default value is 0.0)
        fs >> f;

        // add the newly-converted field to the end of the record
        record.push_back(f);
    }

    // Now we have read a single line, converted into a list of fields, converted the fields
    // from strings to doubles, and stored the results in the argument record, so
    // we just return the argument stream as required for this kind of input overload function.
    return ins;
}

//-----------------------------------------------------------------------------
// Let's likewise overload the stream input operator to read a list of CSV records.
// This time it is a little easier, just because we only need to worry about reading
// records, and not fields.

istream& operator >>(istream& ins, data_t& data) {
    // make sure that the returned data only contains the CSV data we read here
    data.clear();

    // For every record we can read from the file, append it to our resulting data
    record_t record;
    while (ins >> record) {
        data.push_back(record);
    }

    // Again, return the argument stream as required for this kind of input stream overload.
    return ins;
}


geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

int read_traj_file(string fname, nav_msgs::Path &nav_path) {
    //open the trajectory file:
    ifstream infile(fname.c_str());
    if (infile.is_open()) {
        ROS_INFO("opened file");
    }
    if (!infile) // file couldn't be opened
    {
        cerr << "Error: file " << fname << " could not be opened" << endl;
        return 1;
    }
    cout << "opened file " << fname << endl;


    // Here is the data we want.
    data_t data;

    // Here is the file containing the data. Read it into data.
    infile >> data;

    // Complain if something went wrong.
    if (!infile.eof()) {
        cout << "error reading file!\n";
        return 1;
    }

    infile.close();


    // Otherwise, list some basic information about the file.
    cout << "CSV file contains " << data.size() << " records.\n";

    unsigned min_record_size = data[0].size();
    unsigned max_record_size = 0;
    for (unsigned n = 0; n < data.size(); n++) {
        if (max_record_size < data[ n ].size())
            max_record_size = data[ n ].size();
        if (min_record_size > data[ n ].size())
            min_record_size = data[ n ].size();
    }
    if (max_record_size > 3) {
        ROS_WARN("bad file");
        cout << "The largest record has " << max_record_size << " fields.\n";
        return 1;

    }
    if (min_record_size < 3) {
        ROS_WARN("bad file");
        cout << "The smallest record has " << min_record_size << " fields.\n";
        return 1;

    }
    nav_path.poses.clear(); // clear existing data 
    nav_path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose_stamped;
    geometry_msgs::Pose pose;
    pose.position.x = 0.0; // say desired x-coord is 1
    pose.position.y = 0.0;
    pose.position.z = 0.0; // let's hope so!
    pose.orientation.x = 0.0; //always, for motion in horizontal plane
    pose.orientation.y = 0.0; // ditto
    pose.orientation.z = 0.0; // implies oriented at yaw=0, i.e. along x axis
    pose.orientation.w = 1.0; //sum of squares of all components of unit quaternion is 1
    

    
    for (unsigned n = 0; n < data.size(); n++) {
        // pack trajectory points, one at a time:
            pose.position.x = data[n][0];
            pose.position.y = data[n][1];
            pose.orientation = convertPlanarPhi2Quaternion(data[n][2]);
        	pose_stamped.pose = pose;
    		nav_path.poses.push_back(pose_stamped);
    
    }
    return 0;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "path_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<example_ros_service::PathSrv>("path_service");
    geometry_msgs::Quaternion quat;
    nav_msgs::Path des_path;
    while (!client.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }

    ROS_INFO("connected client to service");
    example_ros_service::PathSrv path_srv;
    std::string ros_ws_path = getenv("ROS_WORKSPACE");
	std::string path_to_file= ros_ws_path+"/src/stdr_path_service/stdr_poses.jsp";
	if(0==read_traj_file(path_to_file.c_str(), des_path)) {
		ROS_INFO("Alright, Alright, Alright");
	}

	read_traj_file(path_to_file.c_str(), des_path);    
    path_srv.request.nav_path= des_path;
    ROS_INFO("Sent to service");
    client.call(path_srv);

    return 0;
}
