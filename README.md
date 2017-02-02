# Roboy Mind

The project implements memory concept for Roboy based on KnowRob package.

## Dependencies

The package depends on KnowRob package so make sure to follow the steps from knowRob_installation.txt and source the workspace:
```
source knowrob_ws/devel/setup.bash
```

## Usage

To run the package launch

```
roslaunch roboy_mind json_prolog.launch 
roslaunch roboy_mind nodes.launch 
```

## Services and messages

The package implements service servers for following services. For further documentation read Doxygen documentation.

```
/roboy_mind/assert_property
/roboy_mind/call_query
/roboy_mind/check_property
/roboy_mind/check_query
/roboy_mind/create_instance
/roboy_mind/find_instances
/roboy_mind/position_of
/roboy_mind/show_instances
/roboy_mind/show_property
/roboy_mind/show_property_value
/roboy_mind/size_of
/roboy_mind/storage_place
/roboy_mind/subclass_of
/roboy_mind/subclasses_of
/roboy_mind/siblings_of

```
