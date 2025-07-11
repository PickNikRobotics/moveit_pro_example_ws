#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <QApplication>
#include <QWidget>
#include <QVBoxLayout>
#include <QSlider>
#include <QLabel>
#include <QTimer>

class SliderPublisher : public QWidget
{
public:
  SliderPublisher(rclcpp::Node::SharedPtr node)
      : QWidget(), node_(node), value_(100)
  {
    setWindowTitle("Slider Publisher");
    auto *layout = new QVBoxLayout(this);
    label_ = new QLabel("Value: 100", this);
    slider_ = new QSlider(Qt::Horizontal, this);
    slider_->setRange(0, 100);
    slider_->setSingleStep(1);
    slider_->setValue(100);
    layout->addWidget(label_);
    layout->addWidget(slider_);
    setLayout(layout);
    connect(slider_, &QSlider::valueChanged, this, &SliderPublisher::onValueChanged);
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &SliderPublisher::publishValue);
    timer_->start(100); // 10 Hz
    publisher_ = node_->create_publisher<std_msgs::msg::Int32>("slider_value", 10);
  }

private slots:
  void onValueChanged(int value)
  {
    value_ = value;
    label_->setText(QString("Value: %1").arg(value));
  }
  void publishValue()
  {
    std_msgs::msg::Int32 msg;
    msg.data = value_;
    publisher_->publish(msg);
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  QSlider *slider_;
  QLabel *label_;
  QTimer *timer_;
  int value_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("slider_publisher_node");
  QApplication app(argc, argv);
  SliderPublisher window(node);
  window.show();
  int ret = app.exec();
  rclcpp::shutdown();
  return ret;
}