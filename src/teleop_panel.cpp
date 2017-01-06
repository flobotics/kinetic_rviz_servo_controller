/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QPushButton>

#include <geometry_msgs/Twist.h>
#include <dynamixel_msgs/JointState.h>
#include <std_msgs/Float64.h>

//#include "drive_widget.h"
#include "teleop_panel.h"

namespace rviz_finger_ctrl
{

// BEGIN_TUTORIAL
// Here is the implementation of the TeleopPanel class.  TeleopPanel
// has these responsibilities:
//
// - Act as a container for GUI elements DriveWidget and QLineEdit.
// - Publish command velocities 10 times per second (whether 0 or not).
// - Saving and restoring internal state from a config file.
//
// We start with the constructor, doing the standard Qt thing of
// passing the optional *parent* argument on to the superclass
// constructor, and also zero-ing the velocities we will be
// publishing.
TeleopPanel::TeleopPanel( QWidget* parent )
  : rviz::Panel( parent )
  , linear_velocity_( 0 )
  , angular_velocity_( 0 )
{
  // Next we lay out the "output topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  //QHBoxLayout* topic_layout = new QHBoxLayout;
  //topic_box_ = new rviz::RosTopicProperty(
   //   "adc Topic", "adc_plus_pi_pub",
   //   ros::message_traits::datatype<std_msgs::Float32MultiArray>(),
   //   "dynamixel_msgs/JointState topic to subscribe to.",
   //   this, SLOT(updateTopic()));  

  //topic_box_ = new rviz::RosTopicProperty("Topic","","","","",this, SLOT(updateTopic()));


  //topic_layout->addWidget( topic_box_ );
  //topic_layout->addWidget( new QLabel( "Output Topic:" ));
  //output_topic_editor_ = new QLineEdit;
  //topic_layout->addWidget( output_topic_editor_ );

  QHBoxLayout* servoctrl1_layout = new QHBoxLayout;
  servo1pos_ = new QPushButton("s1 pos");
  servo1neg_ = new QPushButton("s1 neg");
  servoctrl1_layout->addWidget( servo1pos_ );
  servoctrl1_layout->addWidget( servo1neg_ );

  QHBoxLayout* servoctrl2_layout = new QHBoxLayout;
  servo2pos_ = new QPushButton("s2 pos");
  servo2neg_ = new QPushButton("s2 neg");
  servoctrl2_layout->addWidget( servo2pos_ );
  servoctrl2_layout->addWidget( servo2neg_ );

  QHBoxLayout* servoctrl3_layout = new QHBoxLayout;
  servo3pos_ = new QPushButton("s3 pos");
  servo3neg_ = new QPushButton("s3 neg");
  servoctrl3_layout->addWidget( servo3pos_ );
  servoctrl3_layout->addWidget( servo3neg_ );

  QHBoxLayout* servoctrl4_layout = new QHBoxLayout;
  servo4pos_ = new QPushButton("s4 pos");
  servo4neg_ = new QPushButton("s4 neg");
  servoctrl4_layout->addWidget( servo4pos_ );
  servoctrl4_layout->addWidget( servo4neg_ );

  QHBoxLayout* servoctrl5_layout = new QHBoxLayout;
  servo5pos_ = new QPushButton("s5 pos");
  servo5neg_ = new QPushButton("s5 neg");
  servoctrl5_layout->addWidget( servo5pos_ );
  servoctrl5_layout->addWidget( servo5neg_ );

  QHBoxLayout* servoctrl6_layout = new QHBoxLayout;
  servo6pos_ = new QPushButton("s6 pos");
  servo6neg_ = new QPushButton("s6 neg");
  servoctrl6_layout->addWidget( servo6pos_ );
  servoctrl6_layout->addWidget( servo6neg_ );

  QHBoxLayout* servoctrl7_layout = new QHBoxLayout;
  servo7pos_ = new QPushButton("s7 pos");
  servo7neg_ = new QPushButton("s7 neg");
  servoctrl7_layout->addWidget( servo7pos_ );
  servoctrl7_layout->addWidget( servo7neg_ );

  QHBoxLayout* servoctrl8_layout = new QHBoxLayout;
  servo8pos_ = new QPushButton("s8 pos");
  servo8neg_ = new QPushButton("s8 neg");
  servoctrl8_layout->addWidget( servo8pos_ );
  servoctrl8_layout->addWidget( servo8neg_ );

  // Then create the control widget.
  //drive_widget_ = new DriveWidget;

  // Lay out the topic field above the control widget.
  QVBoxLayout* layout = new QVBoxLayout;
  //layout->addLayout( topic_layout );
  layout->addLayout( servoctrl1_layout );
  layout->addLayout( servoctrl2_layout );
  layout->addLayout( servoctrl3_layout );
  layout->addLayout( servoctrl4_layout );
  layout->addLayout( servoctrl5_layout );
  layout->addLayout( servoctrl6_layout );
  layout->addLayout( servoctrl7_layout );
  layout->addLayout( servoctrl8_layout );
  //layout->addWidget( drive_widget_ );
  setLayout( layout );

  // Create a timer for sending the output.  Motor controllers want to
  // be reassured frequently that they are doing the right thing, so
  // we keep re-sending velocities even when they aren't changing.
  // 
  // Here we take advantage of QObject's memory management behavior:
  // since "this" is passed to the new QTimer as its parent, the
  // QTimer is deleted by the QObject destructor when this TeleopPanel
  // object is destroyed.  Therefore we don't need to keep a pointer
  // to the timer.
  QTimer* output_timer = new QTimer( this );

  // Next we make signal/slot connections.
  //connect( drive_widget_, SIGNAL( outputVelocity( float, float )), this, SLOT( setVel( float, float )));
  //connect( output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));
  //connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));
  connect( servo1pos_, SIGNAL( pressed() ), this, SLOT( send_s1pos() ));
  connect( servo1pos_, SIGNAL( released() ), this, SLOT( stop_servo1() ));  
  connect( servo1neg_, SIGNAL( pressed() ), this, SLOT( send_s1neg() ));
  connect( servo1neg_, SIGNAL( released() ), this, SLOT( stop_servo1() ));

  connect( servo2pos_, SIGNAL( pressed() ), this, SLOT( send_s2pos() ));
  connect( servo2pos_, SIGNAL( released() ), this, SLOT( stop_servo2() ));
  connect( servo2neg_, SIGNAL( pressed() ), this, SLOT( send_s2neg() ));
  connect( servo2neg_, SIGNAL( released() ), this, SLOT( stop_servo2() ));

  connect( servo3pos_, SIGNAL( pressed() ), this, SLOT( send_s3pos() ));
  connect( servo3pos_, SIGNAL( released() ), this, SLOT( stop_servo3() ));
  connect( servo3neg_, SIGNAL( pressed() ), this, SLOT( send_s3neg() ));
  connect( servo3neg_, SIGNAL( released() ), this, SLOT( stop_servo3() ));

  connect( servo4pos_, SIGNAL( pressed() ), this, SLOT( send_s4pos() ));
  connect( servo4pos_, SIGNAL( released() ), this, SLOT( stop_servo4() ));
  connect( servo4neg_, SIGNAL( pressed() ), this, SLOT( send_s4neg() ));
  connect( servo4neg_, SIGNAL( released() ), this, SLOT( stop_servo4() ));

  connect( servo5pos_, SIGNAL( pressed() ), this, SLOT( send_s5pos() ));
  connect( servo5pos_, SIGNAL( released() ), this, SLOT( stop_servo5() ));
  connect( servo5neg_, SIGNAL( pressed() ), this, SLOT( send_s5neg() ));
  connect( servo5neg_, SIGNAL( released() ), this, SLOT( stop_servo5() ));

  connect( servo6pos_, SIGNAL( pressed() ), this, SLOT( send_s6pos() ));
  connect( servo6pos_, SIGNAL( released() ), this, SLOT( stop_servo6() ));
  connect( servo6neg_, SIGNAL( pressed() ), this, SLOT( send_s6neg() ));
  connect( servo6neg_, SIGNAL( released() ), this, SLOT( stop_servo6() ));

  connect( servo7pos_, SIGNAL( pressed() ), this, SLOT( send_s7pos() ));
  connect( servo7pos_, SIGNAL( released() ), this, SLOT( stop_servo7() ));
  connect( servo7neg_, SIGNAL( pressed() ), this, SLOT( send_s7neg() ));
  connect( servo7neg_, SIGNAL( released() ), this, SLOT( stop_servo7() ));

  connect( servo8pos_, SIGNAL( pressed() ), this, SLOT( send_s8pos() ));
  connect( servo8pos_, SIGNAL( released() ), this, SLOT( stop_servo8() ));
  connect( servo8neg_, SIGNAL( pressed() ), this, SLOT( send_s8neg() ));
  connect( servo8neg_, SIGNAL( released() ), this, SLOT( stop_servo8() ));

  // Start the timer.
  output_timer->start( 100 );

  // Make the control widget start disabled, since we don't start with an output topic.
  //drive_widget_->setEnabled( false );
}

void TeleopPanel::stop_servo1()
{
  if( ros::ok() && s1_publisher_ )
  {
    std_msgs::Float64 msg;
    msg.data = 0;
    s1_publisher_.publish( msg );
  }
}

void TeleopPanel::stop_servo2()
{
  if( ros::ok() && s2_publisher_ )
  {
    std_msgs::Float64 msg;
    msg.data = 0;
    s2_publisher_.publish( msg );
  }
}

void TeleopPanel::stop_servo3()
{
  if( ros::ok() && s3_publisher_ )
  {
    std_msgs::Float64 msg;
    msg.data = 0;
    s3_publisher_.publish( msg );
  }
}

void TeleopPanel::stop_servo4()
{
  if( ros::ok() && s4_publisher_ )
  {
    std_msgs::Float64 msg;
    msg.data = 0;
    s4_publisher_.publish( msg );
  }
}

void TeleopPanel::stop_servo5()
{
  if( ros::ok() && s5_publisher_ )
  {
    std_msgs::Float64 msg;
    msg.data = 0;
    s5_publisher_.publish( msg );
  }
}

void TeleopPanel::stop_servo6()
{
  if( ros::ok() && s6_publisher_ )
  {
    std_msgs::Float64 msg;
    msg.data = 0;
    s6_publisher_.publish( msg );
  }
}

void TeleopPanel::stop_servo7()
{
  if( ros::ok() && s7_publisher_ )
  {
    std_msgs::Float64 msg;
    msg.data = 0;
    s7_publisher_.publish( msg );
  }
}

void TeleopPanel::stop_servo8()
{
  if( ros::ok() && s8_publisher_ )
  {
    std_msgs::Float64 msg;
    msg.data = 0;
    s8_publisher_.publish( msg );
  }
}

// setVel() is connected to the DriveWidget's output, which is sent
// whenever it changes due to a mouse event.  This just records the
// values it is given.  The data doesn't actually get sent until the
// next timer callback.
void TeleopPanel::setVel( float lin, float ang )
{
  linear_velocity_ = lin;
  angular_velocity_ = ang;
}

// Read the topic name from the QLineEdit and call setTopic() with the
// results.  This is connected to QLineEdit::editingFinished() which
// fires when the user presses Enter or Tab or otherwise moves focus
// away.
void TeleopPanel::updateTopic()
{
  setTopic( output_topic_editor_->text() );
}

// Set the topic name we are publishing to.
void TeleopPanel::setTopic( const QString& new_topic )
{
  // Only take action if the name has changed.
  if( new_topic != output_topic_ )
  {
    output_topic_ = new_topic;
    // If the topic is the empty string, don't publish anything.
    if( output_topic_ == "" )
    {
      s1_publisher_.shutdown();
    }
    else
    {
      // The old ``velocity_publisher_`` is destroyed by this assignment,
      // and thus the old topic advertisement is removed.  The call to
      // nh_advertise() says we want to publish data on the new topic
      // name.
      s1_publisher_ = nh_.advertise<std_msgs::Float64>("/proximal1_1_controller/command",1);   //output_topic_.toStdString(), 1 );
    }
    // rviz::Panel defines the configChanged() signal.  Emitting it
    // tells RViz that something in this panel has changed that will
    // affect a saved config file.  Ultimately this signal can cause
    // QWidget::setWindowModified(true) to be called on the top-level
    // rviz::VisualizationFrame, which causes a little asterisk ("*")
    // to show in the window's title bar indicating unsaved changes.
    Q_EMIT configChanged();
  }

  // Gray out the control widget when the output topic is empty.
  //drive_widget_->setEnabled( output_topic_ != "" );
}

// Publish the control velocities if ROS is not shutting down and the
// publisher is ready with a valid topic name.
void TeleopPanel::send_s1pos()
{
  s1_publisher_ = nh_.advertise<std_msgs::Float64>("/proximal1_1_controller/command",1);

  if( ros::ok() && s1_publisher_ )
  {
    std_msgs::Float64 msg;
    msg.data = 3;
    s1_publisher_.publish( msg );
  }
}


void TeleopPanel::send_s1neg()
{
  s1_publisher_ = nh_.advertise<std_msgs::Float64>("/proximal1_1_controller/command",1);

  if( ros::ok() && s1_publisher_ )
  {
    std_msgs::Float64 msg;
    msg.data = -3;
    s1_publisher_.publish( msg );
  }
}

void TeleopPanel::send_s2pos()
{
  s2_publisher_ = nh_.advertise<std_msgs::Float64>("/proximal1_2_controller/command",1);

  if( ros::ok() && s2_publisher_ )
  {
    std_msgs::Float64 msg;
    msg.data = 3;
    s2_publisher_.publish( msg );
  }
}


void TeleopPanel::send_s2neg()
{
  s2_publisher_ = nh_.advertise<std_msgs::Float64>("/proximal1_2_controller/command",1);

  if( ros::ok() && s2_publisher_ )
  {
    std_msgs::Float64 msg;
    msg.data = -3;
    s2_publisher_.publish( msg );
  }
}

void TeleopPanel::send_s3pos()
{
  s3_publisher_ = nh_.advertise<std_msgs::Float64>("/proximal2_1_controller/command",1);

  if( ros::ok() && s3_publisher_ )
  {
    std_msgs::Float64 msg;
    msg.data = 3;
    s3_publisher_.publish( msg );
  }
}


void TeleopPanel::send_s3neg()
{
  s3_publisher_ = nh_.advertise<std_msgs::Float64>("/proximal2_1_controller/command",1);

  if( ros::ok() && s3_publisher_ )
  {
    std_msgs::Float64 msg;
    msg.data = -3;
    s3_publisher_.publish( msg );
  }
}

void TeleopPanel::send_s4pos()
{
  s4_publisher_ = nh_.advertise<std_msgs::Float64>("/proximal2_2_controller/command",1);

  if( ros::ok() && s4_publisher_ )
  {
    std_msgs::Float64 msg;
    msg.data = 3;
    s4_publisher_.publish( msg );
  }
}


void TeleopPanel::send_s4neg()
{
  s4_publisher_ = nh_.advertise<std_msgs::Float64>("/proximal2_2_controller/command",1);

  if( ros::ok() && s4_publisher_ )
  {
    std_msgs::Float64 msg;
    msg.data = -3;
    s4_publisher_.publish( msg );
  }
}

void TeleopPanel::send_s5pos()
{
  s5_publisher_ = nh_.advertise<std_msgs::Float64>("/proximal1_1_controller/command",1);

  if( ros::ok() && s5_publisher_ )
  {
    std_msgs::Float64 msg;
    msg.data = 3;
    s5_publisher_.publish( msg );
  }
}


void TeleopPanel::send_s5neg()
{
  s5_publisher_ = nh_.advertise<std_msgs::Float64>("/proximal1_1_controller/command",1);

  if( ros::ok() && s5_publisher_ )
  {
    std_msgs::Float64 msg;
    msg.data = -3;
    s5_publisher_.publish( msg );
  }
}
void TeleopPanel::send_s6pos()
{
  s6_publisher_ = nh_.advertise<std_msgs::Float64>("/proximal1_1_controller/command",1);

  if( ros::ok() && s6_publisher_ )
  {
    std_msgs::Float64 msg;
    msg.data = 3;
    s6_publisher_.publish( msg );
  }
}


void TeleopPanel::send_s6neg()
{
  s6_publisher_ = nh_.advertise<std_msgs::Float64>("/proximal1_1_controller/command",1);

  if( ros::ok() && s6_publisher_ )
  {
    std_msgs::Float64 msg;
    msg.data = -3;
    s6_publisher_.publish( msg );
  }
}
void TeleopPanel::send_s7pos()
{
  s7_publisher_ = nh_.advertise<std_msgs::Float64>("/proximal1_1_controller/command",1);

  if( ros::ok() && s7_publisher_ )
  {
    std_msgs::Float64 msg;
    msg.data = 3;
    s7_publisher_.publish( msg );
  }
}


void TeleopPanel::send_s7neg()
{
  s7_publisher_ = nh_.advertise<std_msgs::Float64>("/proximal1_1_controller/command",1);

  if( ros::ok() && s7_publisher_ )
  {
    std_msgs::Float64 msg;
    msg.data = -3;
    s7_publisher_.publish( msg );
  }
}
void TeleopPanel::send_s8pos()
{
  s8_publisher_ = nh_.advertise<std_msgs::Float64>("/proximal1_1_controller/command",1);

  if( ros::ok() && s8_publisher_ )
  {
    std_msgs::Float64 msg;
    msg.data = 3;
    s8_publisher_.publish( msg );
  }
}


void TeleopPanel::send_s8neg()
{
  s8_publisher_ = nh_.advertise<std_msgs::Float64>("/proximal1_1_controller/command",1);

  if( ros::ok() && s8_publisher_ )
  {
    std_msgs::Float64 msg;
    msg.data = -3;
    s8_publisher_.publish( msg );
  }
}


// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void TeleopPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Topic", output_topic_ );
}

// Load all configuration data for this panel from the given Config object.
void TeleopPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
  if( config.mapGetString( "Topic", &topic ))
  {
    output_topic_editor_->setText( topic );
    updateTopic();
  }
}

} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_finger_ctrl::TeleopPanel,rviz::Panel )
// END_TUTORIAL
