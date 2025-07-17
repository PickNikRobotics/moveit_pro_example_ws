#include <QApplication>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

class SliderPublisher : public QWidget
{
public:
  SliderPublisher(rclcpp::Node::SharedPtr node) : QWidget(), node_(node), value_(100), toggle_state_(true)
  {
    setWindowTitle("Slider Publisher");
    auto* layout = new QVBoxLayout(this);
    label_ = new QLabel("Value: 100", this);
    slider_ = new QSlider(Qt::Horizontal, this);
    slider_->setRange(0, 100);
    slider_->setSingleStep(1);
    slider_->setValue(100);
    toggle_button_ = new QPushButton("Toggle (100)", this);
    layout->addWidget(label_);
    layout->addWidget(slider_);
    layout->addWidget(toggle_button_);
    setLayout(layout);
    connect(slider_, &QSlider::valueChanged, this, &SliderPublisher::onValueChanged);
    connect(toggle_button_, &QPushButton::clicked, this, &SliderPublisher::onToggleClicked);
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &SliderPublisher::publishValue);
    timer_->start(100);  // 10 Hz
    publisher_ = node_->create_publisher<std_msgs::msg::Int32>("slider_value", 10);
  }

private slots:
  void onValueChanged(int value)
  {
    value_ = value;
    label_->setText(QString("Value: %1").arg(value));
  }

  void onToggleClicked()
  {
    if (toggle_state_)
    {
      value_ = 0;
      slider_->setValue(0);
      toggle_button_->setText("Toggle (0)");
    }
    else
    {
      value_ = 100;
      slider_->setValue(100);
      toggle_button_->setText("Toggle (100)");
    }
    toggle_state_ = !toggle_state_;
    label_->setText(QString("Value: %1").arg(value_));
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
  QSlider* slider_;
  QLabel* label_;
  QPushButton* toggle_button_;
  QTimer* timer_;
  int value_;
  bool toggle_state_;
};

int main(int argc, char* argv[])
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
