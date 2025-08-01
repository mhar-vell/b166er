// Copyright 2019-2020 The MathWorks, Inc.
// Common copy functions for movemaster_msg/status
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
#include "movemaster_msg/status.h"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class MOVEMASTER_MSG_EXPORT movemaster_msg_msg_status_common : public MATLABROSMsgInterface<movemaster_msg::status> {
  public:
    virtual ~movemaster_msg_msg_status_common(){}
    virtual void copy_from_struct(movemaster_msg::status* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const movemaster_msg::status* msg, MultiLibLoader loader, size_t size = 1);
};
  void movemaster_msg_msg_status_common::copy_from_struct(movemaster_msg::status* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //joint
        const matlab::data::CharArray joint_arr = arr["Joint"];
        msg->joint = joint_arr.toAscii();
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Joint' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Joint' is wrong type; expected a string.");
    }
    try {
        //setpoint
        const matlab::data::TypedArray<float> setpoint_arr = arr["Setpoint"];
        msg->setpoint = setpoint_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Setpoint' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Setpoint' is wrong type; expected a single.");
    }
    try {
        //pulse_count
        const matlab::data::TypedArray<float> pulse_count_arr = arr["PulseCount"];
        msg->pulse_count = pulse_count_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'PulseCount' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'PulseCount' is wrong type; expected a single.");
    }
    try {
        //error
        const matlab::data::TypedArray<float> error_arr = arr["Error"];
        msg->error = error_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Error' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Error' is wrong type; expected a single.");
    }
    try {
        //output
        const matlab::data::TypedArray<float> output_arr = arr["Output"];
        msg->output = output_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Output' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Output' is wrong type; expected a single.");
    }
    try {
        //control_loop
        const matlab::data::TypedArray<float> control_loop_arr = arr["ControlLoop"];
        msg->control_loop = control_loop_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'ControlLoop' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'ControlLoop' is wrong type; expected a single.");
    }
    try {
        //IsDone
        const matlab::data::TypedArray<bool> IsDone_arr = arr["IsDone"];
        msg->IsDone = IsDone_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'IsDone' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'IsDone' is wrong type; expected a logical.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T movemaster_msg_msg_status_common::get_arr(MDFactory_T& factory, const movemaster_msg::status* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Joint","Setpoint","PulseCount","Error","Output","ControlLoop","IsDone"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("movemaster_msg/status");
    // joint
    auto currentElement_joint = (msg + ctr)->joint;
    outArray[ctr]["Joint"] = factory.createCharArray(currentElement_joint);
    // setpoint
    auto currentElement_setpoint = (msg + ctr)->setpoint;
    outArray[ctr]["Setpoint"] = factory.createScalar(currentElement_setpoint);
    // pulse_count
    auto currentElement_pulse_count = (msg + ctr)->pulse_count;
    outArray[ctr]["PulseCount"] = factory.createScalar(currentElement_pulse_count);
    // error
    auto currentElement_error = (msg + ctr)->error;
    outArray[ctr]["Error"] = factory.createScalar(currentElement_error);
    // output
    auto currentElement_output = (msg + ctr)->output;
    outArray[ctr]["Output"] = factory.createScalar(currentElement_output);
    // control_loop
    auto currentElement_control_loop = (msg + ctr)->control_loop;
    outArray[ctr]["ControlLoop"] = factory.createScalar(currentElement_control_loop);
    // IsDone
    auto currentElement_IsDone = (msg + ctr)->IsDone;
    outArray[ctr]["IsDone"] = factory.createScalar(static_cast<bool>(currentElement_IsDone));
    }
    return std::move(outArray);
  } 
class MOVEMASTER_MSG_EXPORT movemaster_msg_status_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~movemaster_msg_status_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          movemaster_msg_status_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<movemaster_msg::status,movemaster_msg_msg_status_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         movemaster_msg_status_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<movemaster_msg::status,movemaster_msg::status::ConstPtr,movemaster_msg_msg_status_common>>();
  }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(movemaster_msg_msg_status_common, MATLABROSMsgInterface<movemaster_msg::status>)
CLASS_LOADER_REGISTER_CLASS(movemaster_msg_status_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1