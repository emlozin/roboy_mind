#ifndef REASONING_NODE_H
#define REASONING_NODE_H

#endif // REASONING_NODE_H

/*! @file
 *  \brief Header file including RoboyMind class declarations.
 *
 *  Storing all declarations of RoboyMind class methods.
 */

// ROS library
#include <ros/ros.h>
#include <ros/duration.h>

// Prolog
#include <json_prolog/prolog.h>

//LOGIC SERVICES
#include <roboy_mind/srvAssertProperty.h>
#include <roboy_mind/srvCallQuery.h>
#include <roboy_mind/srvCheckProperty.h>
#include <roboy_mind/srvCheckQuery.h>
#include <roboy_mind/srvCreateInstance.h>
#include <roboy_mind/srvFindInstances.h>
#include <roboy_mind/srvPositionOf.h>
#include <roboy_mind/srvShowInstances.h>
#include <roboy_mind/srvShowProperty.h>
#include <roboy_mind/srvShowPropertyValue.h>
#include <roboy_mind/srvSizeOf.h>
#include <roboy_mind/srvStoragePlace.h>
#include <roboy_mind/srvSubclassOf.h>
#include <roboy_mind/srvSubclassesOf.h>
#include <roboy_mind/srvSiblingsOf.h>


#include <common.hpp>

using namespace std;
using namespace json_prolog;

#define SHOW_QUERIES true

class RoboyMind{
public:
    // Constructors
    RoboyMind(ros::NodeHandle);
    ~RoboyMind(){};

    /* *******************
     * Service functions
     * *******************/

    /** Service server for asserting property
     *  @param   req         object of srvAssertProperty::Request service type,
     *  @param   res         object of srvAssertProperty::Response service type,
     *  @param   object      name of the instance that the property should be asserted to, string
     *  @param   property    name of the property, string
     *  @param   instance    value for data properties or name of the instance for object properties, string
     *  @param   data        variable indicating whether the property is of data or object type, bool
     *  @return  result of the service, bool
     */
    bool assertPropertySRV(roboy_mind::srvAssertProperty::Request  &req,roboy_mind::srvAssertProperty::Response &res);
    
    /** Service server for calling queries
     *  @param   req         object of srvCallQuery::Request service type,
     *  @param   res         object of srvCallQuery::Response service type,
     *  @param   query       query that needs to be called, string
     *  @return  result of the service stating whether the query has been called, bool
     */
    bool callQuerySRV(roboy_mind::srvCallQuery::Request  &req,roboy_mind::srvCallQuery::Response &res);

    /** Service server for checking property
     *  @param   req         object of srvCheckProperty::Request service type,
     *  @param   res         object of srvCheckProperty::Response service type,
     *  @param   object      name of the instance that the property should be asserted to, string
     *  @param   property    name of the property, string
     *  @param   instance    value for data properties or name of the instance for object properties, string
     *  @param   data        variable indicating whether the property is of data or object type, bool
     *  @return  result of the service stating whether the property with the given value was asserted, bool
     */
    bool checkPropertySRV(roboy_mind::srvCheckProperty::Request  &req,roboy_mind::srvCheckProperty::Response &res);

    /** Service server for checking queries
     *  @param   req         object of srvCheckQuery::Request service type,
     *  @param   res         object of srvCheckQuery::Response service type,
     *  @param   query       query that needs to be checked, string
     *  @return  result of the service stating whether the query is true or false, bool
     */
    bool checkQuerySRV(roboy_mind::srvCheckQuery::Request  &req,roboy_mind::srvCheckQuery::Response &res);

    /** Service server for instance creation
     *  @param   req            object of srvCreateInstance::Request service type,
     *  @param   res            object of srvCreateInstance::Response service type,
     *  @param   object_class   name of the object class which should be instantiated, string
     *  @param   id             id of the instance, int32
     *  @return  result of the service, bool
     */
    bool createInstanceSRV(roboy_mind::srvCreateInstance::Request  &req,roboy_mind::srvCreateInstance::Response &res);

    /** Service server for finding all instances with a given property
     *  @param   req            object of srvFindInstances::Request service type,
     *  @param   res            object of srvFindInstances::Response service type,
     *  @param   property       name of the property, string
     *  @param   instance       value for data properties or name of the instance for object properties, string
     *  @param   data           variable indicating whether the property is of data or object type, bool
     *  @return  vector of instances for which the property holds
     */
    bool findInstancesSRV(roboy_mind::srvFindInstances::Request  &req,roboy_mind::srvFindInstances::Response &res);

    /** Service server for showing position of the instance
     *  @param   req            object of srvPositionOf::Request service type,
     *  @param   res            object of srvPositionOf::Response service type,
     *  @param   object         name of the instance, string
     *  @return  vector of X-,Y- and Z- coordinates of the object
     */
    bool positionOfSRV(roboy_mind::srvPositionOf::Request  &req,roboy_mind::srvPositionOf::Response &res);

    /** Service server for showing instances
     *  @param   req            object of srvShowInstances::Request service type,
     *  @param   res            object of srvShowInstances::Response service type,
     *  @param   object_class   name of the object class instances of which should be shown, string
     *  @return  vector of instances for which the property holds
     */
    bool showInstancesSRV(roboy_mind::srvShowInstances::Request  &req,roboy_mind::srvShowInstances::Response &res);

    /** Service server for showing instances
     *  @param   req            object of srvShowProperty::Request service type,
     *  @param   res            object of srvShowProperty::Response service type,
     *  @param   object         name of the instance properties of which should be shown, string
     *  @return  vector of properties of the object
     */
    bool showPropertySRV(roboy_mind::srvShowProperty::Request  &req,roboy_mind::srvShowProperty::Response &res);

    /** Service server for showing property values of instances
     *  @param   req            object of srvShowPropertyValue::Request service type,
     *  @param   res            object of srvShowPropertyValue::Response service type,
     *  @param   object         name of the instance properties of which should be shown, string
     *  @param   property       name of the property, string
     *  @return  vector of properties of the object
     */
    bool showPropertyValueSRV(roboy_mind::srvShowPropertyValue::Request  &req,roboy_mind::srvShowPropertyValue::Response &res);

    /** Service server for showing storage places for objects
     *  @param   req            object of srvSizeOf::Request service type,
     *  @param   res            object of srvSizeOf::Response service type,
     *  @param   object         name of the instance, string
     *  @return  boolean stating whether the object is rounded (then it has 2dimensions - height and radius, otherwise 3)
     */
    bool sizeOfSRV(roboy_mind::srvSizeOf::Request  &req,roboy_mind::srvSizeOf::Response &res);

    /** Service server for showing storage places for objects
     *  @param   req            object of srvStoragePlace::Request service type,
     *  @param   res            object of srvStoragePlace::Response service type,
     *  @param   object         name of the instance, string
     *  @return  storage place for the object class
     */
    bool storagePlaceSRV(roboy_mind::srvStoragePlace::Request  &req,roboy_mind::srvStoragePlace::Response &res);

    /** Service server for showing parent class of an object
     *  @param   req            object of srvSubclassOf::Request service type,
     *  @param   res            object of srvSubclassOf::Response service type,
     *  @param   object         name of the instance, string
     *  @return  parent subclass
     */
    bool subclassOfSRV(roboy_mind::srvSubclassOf::Request  &req,roboy_mind::srvSubclassOf::Response &res);

    /** Service server for showing all parent class of an object
     *  @param   req            object of srvSubclassesOf::Request service type,
     *  @param   res            object of srvSubclassesOf::Response service type,
     *  @param   object         name of the instance, string
     *  @return  parent subclasses
     */
    bool subclassesOfSRV(roboy_mind::srvSubclassesOf::Request  &req,roboy_mind::srvSubclassesOf::Response &res);

    /** Service server for showing all sibling class of an object
     *  @param   req            object of srvSiblingsOf::Request service type,
     *  @param   res            object of srvSiblingsOf::Response service type,
     *  @param   object         name of the instance, string
     *  @return  sibling classes
     */
    bool siblingsOfSRV(roboy_mind::srvSiblingsOf::Request  &req,roboy_mind::srvSiblingsOf::Response &res);

private:
    Prolog pl;
    
    string ontology_name;
    string knowrob;
    // Logic services
    ros::ServiceServer assert_property_service;
    ros::ServiceServer call_query_service;
    ros::ServiceServer check_query_service;
    ros::ServiceServer check_property_service;
    ros::ServiceServer create_instance_service;
    ros::ServiceServer find_instances_service;
    ros::ServiceServer position_of_service;
    ros::ServiceServer show_instances_service;
    ros::ServiceServer show_properties_service;
    ros::ServiceServer show_property_value_service;
    ros::ServiceServer size_of_service;
    ros::ServiceServer storage_place_service;
    ros::ServiceServer subclass_of_service;
    ros::ServiceServer subclasses_of_service;
    ros::ServiceServer siblings_of_service;

    /// The node handle
    ros::NodeHandle nh_;
    /// Node handle in the private namespace
    ros::NodeHandle priv_nh_;
};