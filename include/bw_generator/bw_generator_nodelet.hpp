/* Copyright (c) 2017
 * ############################################################################
 *
 * File: bw_generator_nodelet.hpp
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

#ifndef BW_GENERATOR_BWGENERATORNODELET_HPP
#define BW_GENERATOR_BWGENERATORNODELET_HPP

//      ROS INCLUDES
#include <ros/package.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/UInt8MultiArray.h>          // for reading actuation enable status

namespace bw_generator {

class BWGeneratorNodelet : public nodelet::Nodelet {
 public:
    BWGeneratorNodelet();   // constructor
    ~BWGeneratorNodelet();  // destructor

 private:
    virtual void onInit();

    // Usage variables
    ros::NodeHandle public_nh_;      // public node handle
    ros::NodeHandle private_nh_;     // private node handle
    ros::NodeHandle bw_nh_;          // bw node handle

    // Params
    std::string bwgen_name_;
    double msg_rate_;
    int msg_size_;
    int msg_val_;
    std::vector<unsigned char> msg_;

    // Pubs, Timer, Callback
    ros::Publisher bwmsg_pub_;
    ros::Timer t_bwmsg_cb_;
    void bwGenMsgCb();
};  // class BWGeneratorNodelet

}  // namespace bw_generator

#endif  // BW_GENERATOR_BWGENERATORNODELET_HPP
