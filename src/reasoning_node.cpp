/* ***************************************
    Author:	Emilia Lozinska
    E-mail:	ga47nub@mytum.de
*/

/*! @file
 *  \brief Main reasoning node.
 *         Storing all service and callback definitions.
 *
 *  Storing all definitions of RoboyMin dclass methods.
 */
#include <roboy_mind/reasoning_node.h>

/* *******************
* Service functions
* *******************/
bool RoboyMind::assertPropertySRV(roboy_mind::srvAssertProperty::Request  &req,roboy_mind::srvAssertProperty::Response &res)
{
    // Query part for the object
    string query = "rdf_assert('" + this->ontology_name;
    query += req.object;
    query += "','" + this->ontology_name;
    query += req.property;
    if (!req.data){
      query += "','" + this->ontology_name;
      query += req.instance;
    }
    else{
      query += "','";
      query += req.instance;
    }
    query += "')";
    if (SHOW_QUERIES)   
        cout << query << endl;
    PrologQueryProxy bdgs = pl.query(query);
    res.result = true;
    return true;
}

bool RoboyMind::callQuerySRV(roboy_mind::srvCallQuery::Request  &req,roboy_mind::srvCallQuery::Response &res)
{
    // Quering Prolog to see the instances
    PrologQueryProxy bdgs = pl.query(req.query);
    if (SHOW_QUERIES) 
        cout << req.query << endl;
    res.result = false;
    for(PrologQueryProxy::iterator it=bdgs.begin();it != bdgs.end(); it++)
    {
        PrologBindings bdg = *it;
        //cout << "Query is : " << (bool)(it == bdgs.end()) << endl;
        res.result = true;
    }
    return true;
}


bool RoboyMind::checkPropertySRV(roboy_mind::srvCheckProperty::Request  &req,roboy_mind::srvCheckProperty::Response &res)
{
    // Query part for the object
    string query = "owl_has('" + this->ontology_name;
    query += req.object;
    query += "','" + this->ontology_name;
    query += req.property;
    if (!req.data)
      query += "','" + this->ontology_name;
    else
      query += "','";
    query += req.instance;
    query += "')";
    if (SHOW_QUERIES) 
        cout << query << endl;
    PrologQueryProxy bdgs = pl.query(query);
    res.result = false;
    for(PrologQueryProxy::iterator it=bdgs.begin();it != bdgs.end(); it++)
    {
        PrologBindings bdg = *it;
        //cout << "Real query is : " << (bool)(it == bdgs.end()) << endl;
        res.result = true;
    }
}

bool RoboyMind::checkQuerySRV(roboy_mind::srvCheckQuery::Request  &req,roboy_mind::srvCheckQuery::Response &res)
{
    // Quering Prolog to see the instances
    PrologQueryProxy bdgs = pl.query(req.query);
    if (SHOW_QUERIES) 
        cout << req.query << endl;
    res.result = false;
    for(PrologQueryProxy::iterator it=bdgs.begin();it != bdgs.end(); it++)
    {
        PrologBindings bdg = *it;
        //cout << "Query is : " << (bool)(it == bdgs.end()) << endl;
        res.result = true;
    }
    return true;
}

bool RoboyMind::createInstanceSRV(roboy_mind::srvCreateInstance::Request  &req,roboy_mind::srvCreateInstance::Response &res)
{
    // Building the query
    string query = "create_instance_from_class('" + this->ontology_name;
    query += req.object_class;
    query += "',";
    query += IntToStr(req.id);
    query += ", ObjInst)";
    if (SHOW_QUERIES) 
        cout << query << endl;
    PrologQueryProxy bdgs = pl.query(query);
    res.instance = req.object_class + "_" + IntToStr(req.id);
    ROS_INFO("New instance created. \n Instance id: %d ", req.id);
    return true;
}

bool RoboyMind::findInstancesSRV(roboy_mind::srvFindInstances::Request  &req,roboy_mind::srvFindInstances::Response &res)
{
    vector<string> result;
    // Query part for the object
    string query = "owl_has(A,'" + this->ontology_name;
    query += req.property;
    query += "','";
    if (!req.data){
      query += "" + this->ontology_name;
    }
    query += req.value;
    query += "')";
    if (SHOW_QUERIES)   
        cout << query << endl;
    PrologQueryProxy bdgs = pl.query(query);
    stringstream ss;
    for(PrologQueryProxy::iterator it=bdgs.begin();it != bdgs.end(); it++)
    {
        PrologBindings bdg = *it;
        //cout << "A : " << bdg["A"] << endl;
        ss << bdg["A"];
        result.push_back(ss.str());
        ss.str(std::string());
    }
    res.instances = result;
    return true;
}

bool RoboyMind::positionOfSRV(roboy_mind::srvPositionOf::Request  &req,roboy_mind::srvPositionOf::Response &res)
{
    vector<float> position;

    string query = "owl_has('" + ontology_name + req.object + "','" + ontology_name + "xCoord',P)";
    if (SHOW_QUERIES) 
        cout << query << endl;
    // Quering Prolog to see the X - coordinate
    PrologQueryProxy bdgs = pl.query(query);
    stringstream ss;
    for(PrologQueryProxy::iterator it=bdgs.begin();it != bdgs.end(); it++)
    {
        PrologBindings bdg = *it;
        cout << "X - coordinate: " << bdg["P"] << endl;
        ss << bdg["P"];
        position.push_back(StrToFloat(ss.str()));
        ss.str(std::string());
    }   
    query = "owl_has('" + ontology_name + req.object + "','" + ontology_name + "yCoord',P)";
    if (SHOW_QUERIES) 
        cout << query << endl;
    // Quering Prolog to see the Y - coordinate
    bdgs = pl.query(query);
    for(PrologQueryProxy::iterator it=bdgs.begin();it != bdgs.end(); it++)
    {
        PrologBindings bdg = *it;
        cout << "Y - coordinate: " << bdg["P"] << endl;
        ss << bdg["P"];
        position.push_back(StrToFloat(ss.str()));
        ss.str(std::string());
    }   
    query = "owl_has('" + ontology_name + req.object + "','" + ontology_name + "zCoord',P)";
    if (SHOW_QUERIES) 
        cout << query << endl;
    // Quering Prolog to see the Z - coordinate
    bdgs = pl.query(query);
    for(PrologQueryProxy::iterator it=bdgs.begin();it != bdgs.end(); it++)
    {
        PrologBindings bdg = *it;
        cout << "Z - coordinate: " << bdg["P"] << endl;
        ss << bdg["P"];
        position.push_back(StrToFloat(ss.str()));
        ss.str(std::string());
    }
    res.position = position;
    return true;
}

bool RoboyMind::showInstancesSRV(roboy_mind::srvShowInstances::Request  &req,roboy_mind::srvShowInstances::Response &res)
{
    vector<string> result;
    string query = "rdfs_individual_of(I, '" + this->ontology_name;
    query += req.object_class;
    query += "')";
    if (SHOW_QUERIES) 
        cout << query << endl;
    // Quering Prolog to see the instances
    PrologQueryProxy bdgs = pl.query(query);
    stringstream ss;
    for(PrologQueryProxy::iterator it=bdgs.begin();it != bdgs.end(); it++)
    {
        PrologBindings bdg = *it;
        //cout << "I : " << bdg["I"] << endl;
        ss << bdg["I"];
        result.push_back(ss.str());
        ss.str(std::string());
    }
    res.instances = result;
    return true;
}

bool RoboyMind::showPropertySRV(roboy_mind::srvShowProperty::Request  &req,roboy_mind::srvShowProperty::Response &res)
{
    vector<string> result;
    // Query part for the object
    string query = "owl_has('";
    query += ontology_name;
    query += req.object;
    query += "',A,P)";
    if (SHOW_QUERIES) 
        cout << query << endl;
    // Quering Prolog to see the properties and values
    PrologQueryProxy bdgs = pl.query(query);
    stringstream ss;
    for(PrologQueryProxy::iterator it=bdgs.begin();it != bdgs.end(); it++)
    {
        PrologBindings bdg = *it;
        cout << "Property : " << bdg["A"] << endl;
        cout << "Value : " << bdg["P"] << endl;
        ss << bdg["A"];
        result.push_back(ss.str());
        ss.str(std::string());
    }
    res.property = result;
    return true;
}

bool RoboyMind::showPropertyValueSRV(roboy_mind::srvShowPropertyValue::Request  &req,roboy_mind::srvShowPropertyValue::Response &res)
{
    vector<string> result;
    // Query part for the object
    string query = "owl_has('";
    query += req.object;
    query += "','" + this->ontology_name;
    query += req.property;
    query += "',P)";
    if (SHOW_QUERIES)   
        cout << query << endl;
    // Quering Prolog to see the properties and values
    PrologQueryProxy bdgs = pl.query(query);
    stringstream ss;
    for(PrologQueryProxy::iterator it=bdgs.begin();it != bdgs.end(); it++)
    {
        PrologBindings bdg = *it;
        cout << "Value : " << bdg["P"] << endl;
        ss << bdg["P"];
        result.push_back(ss.str());
        ss.str(std::string());
    }
    res.value = result;
    return true;
}

bool RoboyMind::sizeOfSRV(roboy_mind::srvSizeOf::Request  &req,roboy_mind::srvSizeOf::Response &res)
{
    // vector<float> size;
    // Query part for the object
    string query = "owl_has('" + ontology_name + req.object + "','" + ontology_name + "widthOfObject',P)";
    // Quering Prolog to see the properties and values
    PrologQueryProxy bdgs = pl.query(query);
    if (SHOW_QUERIES) 
        cout << query << endl;
    stringstream ss;
    for(PrologQueryProxy::iterator it=bdgs.begin();it != bdgs.end(); it++)
    {
        PrologBindings bdg = *it;
        cout << "Value : " << bdg["P"] << endl;
        ss << bdg["P"];
        res.size.push_back(StrToFloat(ss.str()));
        ss.str(std::string());
    }
    if (res.size.empty())
    {
        query = "owl_has('" + ontology_name + req.object + "','" + ontology_name + "radius',P)";
        if (SHOW_QUERIES) 
            cout << query << endl;
        // Quering Prolog to see the properties and values
        bdgs = pl.query(query);
        for(PrologQueryProxy::iterator it=bdgs.begin();it != bdgs.end(); it++)
        {
            PrologBindings bdg = *it;
            cout << "Value : " << bdg["P"] << endl;
            ss << bdg["P"];
            res.size.push_back(StrToFloat(ss.str()));
            ss.str(std::string());
        }   
        query = "owl_has('" + ontology_name + req.object + "','" + ontology_name + "heightOfObject',P)";
        if (SHOW_QUERIES) 
            cout << query << endl;
        // Quering Prolog to see the properties and values
        bdgs = pl.query(query);
        for(PrologQueryProxy::iterator it=bdgs.begin();it != bdgs.end(); it++)
        {
            PrologBindings bdg = *it;
            cout << "Value : " << bdg["P"] << endl;
            ss << bdg["P"];
            res.size.push_back(StrToFloat(ss.str()));
            ss.str(std::string());
        }   
        res.radius = true;
    }
    else
    {
        query = "owl_has('"+ ontology_name + req.object + "','" + ontology_name + "depthOfObject',P)";
        if (SHOW_QUERIES) 
            cout << query << endl;
        // Quering Prolog to see the properties and values
        PrologQueryProxy bdgs = pl.query(query);
        for(PrologQueryProxy::iterator it=bdgs.begin();it != bdgs.end(); it++)
        {
            PrologBindings bdg = *it;
            cout << "Value : " << bdg["P"] << endl;
            ss << bdg["P"];
            res.size.push_back(StrToFloat(ss.str()));
            ss.str(std::string());
        }   
        query = "owl_has('" + ontology_name + req.object + "','" + ontology_name + "heightOfObject',P)";
        if (SHOW_QUERIES) 
            cout << query << endl;
        // Quering Prolog to see the properties and values
        bdgs = pl.query(query);
        for(PrologQueryProxy::iterator it=bdgs.begin();it != bdgs.end(); it++)
        {
            PrologBindings bdg = *it;
            cout << "Value : " << bdg["P"] << endl;
            ss << bdg["P"];
            res.size.push_back(StrToFloat(ss.str()));
            ss.str(std::string());
        }   
        res.radius = false;
    }
    // res.size = size;
    return true;
}

bool RoboyMind::storagePlaceSRV(roboy_mind::srvStoragePlace::Request  &req,roboy_mind::srvStoragePlace::Response &res)
{
    vector<string> result;
    // Query part for the object
    string query = "storagePlaceFor(Place,'" + this->ontology_name + req.object + "')";
    if (SHOW_QUERIES) 
        cout << query << endl;
    // Quering Prolog to see the properties and values
    PrologQueryProxy bdgs = pl.query(query);
    stringstream ss;
    for(PrologQueryProxy::iterator it=bdgs.begin();it != bdgs.end(); it++)
    {
        PrologBindings bdg = *it;
        ss << bdg["Place"];
        if (std::find(result.begin(), result.end(), ss.str()) == result.end())
            result.push_back(ss.str());
        ss.str(std::string());
    }
    res.storage = result;
    return true;
}

bool RoboyMind::subclassOfSRV(roboy_mind::srvSubclassOf::Request  &req,roboy_mind::srvSubclassOf::Response &res)
{
    // STEP 1: Get object class
    vector<string> obj_class;
    // Query part for the object
    string query = "rdfs_individual_of('" + this->ontology_name + req.object + "',Class)";
    if (SHOW_QUERIES) 
        cout << query << endl;
    // Quering Prolog to see the properties and values
    PrologQueryProxy bdgs = pl.query(query);
    stringstream ss;
    for(PrologQueryProxy::iterator it=bdgs.begin();it != bdgs.end(); it++)
    {
        PrologBindings bdg = *it;
        ss << bdg["Class"];
        if (std::find(obj_class.begin(), obj_class.end(), ss.str()) == obj_class.end())
            obj_class.push_back(ss.str());
        ss.str(std::string());
    }
    // STEP 2: Get parent class
    vector<string> result;
    // Query part for the object
    query = "owl_subclass_of('" + obj_class[0] + "',Parent)";
    if (SHOW_QUERIES) 
        cout << query << endl;
    // Quering Prolog to see the properties and values
    bdgs = pl.query(query);
    for(PrologQueryProxy::iterator it=bdgs.begin();it != bdgs.end(); it++)
    {
        PrologBindings bdg = *it;
        ss << bdg["Parent"];
        if (std::find(result.begin(), result.end(), ss.str()) == result.end())
            result.push_back(ss.str());
        ss.str(std::string());
    }
    res.superclass = result[1];
    return true;
}

bool RoboyMind::subclassesOfSRV(roboy_mind::srvSubclassesOf::Request  &req,roboy_mind::srvSubclassesOf::Response &res)
{
    // STEP 1: Get object class
    vector<string> obj_class;
    // Query part for the object
    string query = "rdfs_individual_of('" + this->ontology_name + req.object + "',Class)";
    if (SHOW_QUERIES) 
        cout << query << endl;
    // Quering Prolog to see the properties and values
    PrologQueryProxy bdgs = pl.query(query);
    stringstream ss;
    for(PrologQueryProxy::iterator it=bdgs.begin();it != bdgs.end(); it++)
    {
        PrologBindings bdg = *it;
        ss << bdg["Class"];
        obj_class.push_back(ss.str());
        ss.str(std::string());
    }
    // STEP 2: Get parent class
    vector<string> result;
    // Query part for the object
    query = "owl_subclass_of('" + obj_class[0] + "',Parent)";
    if (SHOW_QUERIES) 
        cout << query << endl;
    // Quering Prolog to see the properties and values
    bdgs = pl.query(query);
    for(PrologQueryProxy::iterator it=bdgs.begin();it != bdgs.end(); it++)
    {
        PrologBindings bdg = *it;
        ss << bdg["Parent"];
        if (std::find(result.begin(), result.end(), ss.str()) == result.end())
            result.push_back(ss.str());
        ss.str(std::string());
    }
    res.superclass = result;
    return true;
}

bool RoboyMind::siblingsOfSRV(roboy_mind::srvSiblingsOf::Request  &req,roboy_mind::srvSiblingsOf::Response &res)
{
    // STEP 1: Get object class
    vector<string> obj_class;
    // Query part for the object
    string query = "rdfs_individual_of('" + this->ontology_name + req.object + "',Class)";
    if (SHOW_QUERIES) 
        cout << query << endl;
    // Quering Prolog to see the properties and values
    PrologQueryProxy bdgs = pl.query(query);
    stringstream ss;
    for(PrologQueryProxy::iterator it=bdgs.begin();it != bdgs.end(); it++)
    {
        PrologBindings bdg = *it;
        ss << bdg["Class"];
        obj_class.push_back(ss.str());
        ss.str(std::string());
    }
    // STEP 2: Get parent class
    vector<string> result;
    // Query part for the object
    query = "owl_subclass_of('" + obj_class[0] + "',Parent)";
    if (SHOW_QUERIES) 
        cout << query << endl;
    // Quering Prolog to see the properties and values
    bdgs = pl.query(query);
    for(PrologQueryProxy::iterator it=bdgs.begin();it != bdgs.end(); it++)
    {
        PrologBindings bdg = *it;
        ss << bdg["Parent"];
        result.push_back(ss.str());
        ss.str(std::string());
    }
    // STEP 3: Get siblings
    vector<string> instances;
    query = "rdfs_individual_of(Siblings, '" +result[1] + "')";
    if (SHOW_QUERIES) 
        cout << query << endl;
    // Quering Prolog to see the instances
    bdgs = pl.query(query);
    for(PrologQueryProxy::iterator it=bdgs.begin();it != bdgs.end(); it++)
    {
        PrologBindings bdg = *it;
        ss << bdg["Siblings"];
        if (ss.str() != (this->ontology_name + req.object))
            if (std::find(instances.begin(), instances.end(), ss.str()) == instances.end())
                instances.push_back(ss.str());
        ss.str(std::string());
    }
    res.siblings = instances;
    return true;
}


// Constructor
RoboyMind::RoboyMind(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
{
    // Service Servers
    assert_property_service = nh_.advertiseService("/roboy_mind/assert_property",&RoboyMind::assertPropertySRV,this);
    call_query_service = nh_.advertiseService("/roboy_mind/call_query",&RoboyMind::callQuerySRV,this);
    check_property_service = nh_.advertiseService("/roboy_mind/check_property",&RoboyMind::checkPropertySRV,this);
    check_query_service = nh_.advertiseService("/roboy_mind/check_query",&RoboyMind::checkQuerySRV,this);
    create_instance_service = nh_.advertiseService("/roboy_mind/create_instance",&RoboyMind::createInstanceSRV,this);
    find_instances_service = nh_.advertiseService("/roboy_mind/find_instances",&RoboyMind::findInstancesSRV,this);
    position_of_service = nh_.advertiseService("/roboy_mind/position_of",&RoboyMind::positionOfSRV,this);
    show_instances_service = nh_.advertiseService("/roboy_mind/show_instances",&RoboyMind::showInstancesSRV,this);
    show_properties_service = nh_.advertiseService("/roboy_mind/show_property",&RoboyMind::showPropertySRV,this);
    show_property_value_service = nh_.advertiseService("/roboy_mind/show_property_value",&RoboyMind::showPropertyValueSRV,this);
    size_of_service = nh_.advertiseService("/roboy_mind/size_of",&RoboyMind::sizeOfSRV,this);
    storage_place_service = nh_.advertiseService("/roboy_mind/storage_place",&RoboyMind::storagePlaceSRV,this);
    subclass_of_service = nh_.advertiseService("/roboy_mind/subclass_of",&RoboyMind::subclassOfSRV,this);
    subclasses_of_service = nh_.advertiseService("/roboy_mind/subclasses_of",&RoboyMind::subclassesOfSRV,this);
    siblings_of_service = nh_.advertiseService("/roboy_mind/siblings_of",&RoboyMind::siblingsOfSRV,this);

    //nh.param<std::string>("/knowrob", knowrob, "http://knowrob.org/kb/knowrob.owl#");
    nh.param<std::string>("/ontology_name", ontology_name, "http://knowrob.org/kb/knowrob.owl#");//"http://knowrob.org/kb/semRoom_semantic_map.owl#");
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "reasoning_node");
    ros::NodeHandle nh;
    RoboyMind node(nh);
    ROS_INFO("Reasoning node init successful");
    ros::spin();
    return 0;
}