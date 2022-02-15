#ifndef pipeline_images_h
#define pipeline_images_h

#include <ros/ros.h>

#include <image_transport/publisher.h>
#include <image_transport/subscriber.h>
#include <image_transport/image_transport.h>

template <class MessageType>
class PipelineOutputImage : public PipelineConnectorOutput {
public:

  typedef boost::shared_ptr<const MessageType> MessageTypePtr;

  PipelineOutputImage() {
    MessageTypePtr last_message_(new MessageType);
  }
  ~PipelineOutputImage() {}

  void Initialise(std::string topic, std::vector<PipelineConnectorOutput*> &pipe_container) {
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    subscriber_ = it.subscribe(topic, 1, &PipelineOutputImage::SubscriberCallback, this);
    pipe_container.push_back(this);
  }

  MessageTypePtr last_message;

private:

  void SubscriberCallback(const MessageTypePtr &message) {
    last_message = message;
    message_received_ = true;
  }


  std::string topic_;
  image_transport::Subscriber subscriber_;
};


template <class MessageType>
class PipelineInputImage {
public:

  PipelineInputImage() {}
  ~PipelineInputImage() {}

  typedef boost::shared_ptr<const MessageType> MessageTypePtr;


  void Initialise(std::string topic) {
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    publisher_ = it.advertise(topic, 1);
  }

  void PublishMessage(MessageTypePtr message) {
    publisher_.publish(message);
  }

private:

  std::string topic_;
  image_transport::Publisher publisher_;

};




#endif
