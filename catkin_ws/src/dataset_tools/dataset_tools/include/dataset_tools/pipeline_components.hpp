#ifndef pipeline_components_h
#define pipeline_components_h

#include <ros/ros.h>


template <class MessageType>
class PipelineInput {
public:

  PipelineInput() {}
  ~PipelineInput() {}

  typedef boost::shared_ptr<const MessageType> MessageTypePtr;


  void Initialise(std::string topic) {
    ros::NodeHandle nh;
    publisher_ = nh.advertise<MessageType>(topic, 1);
  }

  void PublishMessage(MessageTypePtr message) {
    publisher_.publish(message);
  }

private:

  std::string topic_;
  ros::Publisher publisher_;

};

class PipelineConnectorOutput {
public:

  PipelineConnectorOutput(): message_received_(false) {

  }

  bool message_received_;

};


template <class MessageType>
class PipelineOutput : public PipelineConnectorOutput {
public:

  typedef boost::shared_ptr<MessageType> MessageTypePtr;

  PipelineOutput() {
    MessageTypePtr last_message_(new MessageType);
  }
  ~PipelineOutput() {}

  void Initialise(std::string topic, std::vector<PipelineConnectorOutput*> &pipe_container) {
    ros::NodeHandle nh;
    subscriber_ = nh.subscribe<MessageTypePtr>(topic, 1, &PipelineOutput::SubscriberCallback, this);
    pipe_container.push_back(this);
  }

  MessageTypePtr last_message;

private:

  void SubscriberCallback(const MessageTypePtr message) {
    last_message = message;
    message_received_ = true;
  }


  std::string topic_;
  ros::Subscriber subscriber_;
};



#endif
