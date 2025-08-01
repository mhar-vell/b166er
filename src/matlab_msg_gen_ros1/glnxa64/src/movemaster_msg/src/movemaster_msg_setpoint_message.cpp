// Copyright 2019-2020 The MathWorks, Inc.
// Common copy functions for movemaster_msg/setpoint
#include "boost/date_time.hpp"
#include "boost/shared_array.hpp"
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4244)
#pragma warning(disable : 4265)
#pragma warning(disable : 4458)
#pragma warning(disable : 4100)
#pragma warning(disable : 4127)
#pragma warning(disable : 4267)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wshadow"
#endif //_MSC_VER
#include "ros/ros.h"
#include "movemaster_msg/setpoint.h"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class MOVEMASTER_MSG_EXPORT movemaster_msg_msg_setpoint_common : public MATLABROSMsgInterface<movemaster_msg::setpoint> {
  public:
    virtual ~movemaster_msg_msg_setpoint_common(){}
    virtual void copy_from_struct(movemaster_msg::setpoint* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const movemaster_msg::setpoint* msg, MultiLibLoader loader, size_t size = 1);
};
  void movemaster_msg_msg_setpoint_common::copy_from_struct(movemaster_msg::setpoint* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //set_1
        const matlab::data::TypedArray<float> set_1_arr = arr["Set1"];
        msg->set_1 = set_1_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Set1' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Set1' is wrong type; expected a single.");
    }
    try {
        //set_2
        const matlab::data::TypedArray<float> set_2_arr = arr["Set2"];
        msg->set_2 = set_2_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Set2' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Set2' is wrong type; expected a single.");
    }
    try {
        //set_3
        const matlab::data::TypedArray<float> set_3_arr = arr["Set3"];
        msg->set_3 = set_3_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Set3' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Set3' is wrong type; expected a single.");
    }
    try {
        //set_4
        const matlab::data::TypedArray<float> set_4_arr = arr["Set4"];
        msg->set_4 = set_4_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Set4' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Set4' is wrong type; expected a single.");
    }
    try {
        //set_5
        const matlab::data::TypedArray<float> set_5_arr = arr["Set5"];
        msg->set_5 = set_5_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Set5' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Set5' is wrong type; expected a single.");
    }
    try {
        //set_GRIP
        const matlab::data::TypedArray<bool> set_GRIP_arr = arr["SetGRIP"];
        msg->set_GRIP = set_GRIP_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'SetGRIP' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'SetGRIP' is wrong type; expected a logical.");
    }
    try {
        //emergency_stop
        const matlab::data::TypedArray<bool> emergency_stop_arr = arr["EmergencyStop"];
        msg->emergency_stop = emergency_stop_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'EmergencyStop' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'EmergencyStop' is wrong type; expected a logical.");
    }
    try {
        //GoHome
        const matlab::data::TypedArray<int8_t> GoHome_arr = arr["GoHome"];
        msg->GoHome = GoHome_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'GoHome' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'GoHome' is wrong type; expected a int8.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T movemaster_msg_msg_setpoint_common::get_arr(MDFactory_T& factory, const movemaster_msg::setpoint* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Set1","Set2","Set3","Set4","Set5","SetGRIP","EmergencyStop","GoHome"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("movemaster_msg/setpoint");
    // set_1
    auto currentElement_set_1 = (msg + ctr)->set_1;
    outArray[ctr]["Set1"] = factory.createScalar(currentElement_set_1);
    // set_2
    auto currentElement_set_2 = (msg + ctr)->set_2;
    outArray[ctr]["Set2"] = factory.createScalar(currentElement_set_2);
    // set_3
    auto currentElement_set_3 = (msg + ctr)->set_3;
    outArray[ctr]["Set3"] = factory.createScalar(currentElement_set_3);
    // set_4
    auto currentElement_set_4 = (msg + ctr)->set_4;
    outArray[ctr]["Set4"] = factory.createScalar(currentElement_set_4);
    // set_5
    auto currentElement_set_5 = (msg + ctr)->set_5;
    outArray[ctr]["Set5"] = factory.createScalar(currentElement_set_5);
    // set_GRIP
    auto currentElement_set_GRIP = (msg + ctr)->set_GRIP;
    outArray[ctr]["SetGRIP"] = factory.createScalar(static_cast<bool>(currentElement_set_GRIP));
    // emergency_stop
    auto currentElement_emergency_stop = (msg + ctr)->emergency_stop;
    outArray[ctr]["EmergencyStop"] = factory.createScalar(static_cast<bool>(currentElement_emergency_stop));
    // GoHome
    auto currentElement_GoHome = (msg + ctr)->GoHome;
    outArray[ctr]["GoHome"] = factory.createScalar(currentElement_GoHome);
    }
    return std::move(outArray);
  } 
class MOVEMASTER_MSG_EXPORT movemaster_msg_setpoint_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~movemaster_msg_setpoint_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          movemaster_msg_setpoint_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<movemaster_msg::setpoint,movemaster_msg_msg_setpoint_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         movemaster_msg_setpoint_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<movemaster_msg::setpoint,movemaster_msg::setpoint::ConstPtr,movemaster_msg_msg_setpoint_common>>();
  }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(movemaster_msg_msg_setpoint_common, MATLABROSMsgInterface<movemaster_msg::setpoint>)
CLASS_LOADER_REGISTER_CLASS(movemaster_msg_setpoint_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1