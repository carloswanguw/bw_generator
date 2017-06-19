/* Copyright (c) 2017
 * ############################################################################
 *
 * File: bw_generator_nodelet.cpp
 * Desc: Bandwidth Generator
 * Auth: Carlos Wang
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE WATERLOO AUTONOMOUS VEHICLES
 * LABORATORY BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * ############################################################################
 */

#include "bw_generator/bw_generator_nodelet.hpp"

namespace bw_generator {

BWGeneratorNodelet::BWGeneratorNodelet() {}
BWGeneratorNodelet::~BWGeneratorNodelet() {}

// onInit() - on the initialization of the nodelet (not the class)
void BWGeneratorNodelet::onInit() {
    // Report start of function
    NODELET_INFO("Initializing Nodelet ... ");

    // get public/private node handle
    this->public_nh_ = this->getNodeHandle();
    this->private_nh_ = this->getPrivateNodeHandle();

    // get relevant parameters
    std::string err_string = "BAD_STRING";
    this->private_nh_.param("bw_node_name", this->bwgen_name_, err_string);
    ROS_INFO_STREAM("bw_node_name: " << this->bwgen_name_);
    this->private_nh_.param("bw_msg_rate", this->msg_rate_, -1.0);
    ROS_INFO_STREAM("bw_rate: " << this->msg_rate_);
    this->private_nh_.param("bw_msg_size", this->msg_size_, -1);
    ROS_INFO_STREAM("bw_msg_size: " << this->msg_size_);
    this->private_nh_.param("bw_msg_val", this->msg_val_, -1);
    ROS_INFO_STREAM("bw_msg_val: " << this->msg_val_);

    // generate fake message and store it
    std::vector<unsigned char> dmy_msg(this->msg_size_, this->msg_val_);
    this->msg_ = dmy_msg;

    // create new node handle
    ros::NodeHandle nh(std::string("/") + this->bwgen_name_);
    this->bw_nh_ = nh;

    // declare publisher and timers
    this->bwmsg_pub_ = this->bw_nh_.advertise<std_msgs::UInt8MultiArray>
                ("dummy_msg", 0);
    this->t_bwmsg_cb_ = this->public_nh_.createTimer(
                ros::Duration(1.0/this->msg_rate_),
                boost::bind(&BWGeneratorNodelet::bwGenMsgCb, this));

    // Report end of function
    NODELET_DEBUG("... Nodelet Loaded. ");
}

// Message Generator Callback
void BWGeneratorNodelet::bwGenMsgCb() {
    std_msgs::UInt8MultiArray msg;      // create message array
    msg.data = this->msg_;              // store data into array
    this->bwmsg_pub_.publish(msg);      // publish message
}
}  // namespace bw_generator

PLUGINLIB_DECLARE_CLASS(bw_generator, BWGeneratorNodelet,
                        bw_generator::BWGeneratorNodelet,
                        nodelet::Nodelet);
