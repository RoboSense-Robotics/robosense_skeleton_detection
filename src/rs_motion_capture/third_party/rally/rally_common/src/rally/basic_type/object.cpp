/******************************************************************************
 * Copyright 2017 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include "rally/common/basic_type/object.h"

namespace rally {

void Object3D::transform(const Eigen::Matrix4d& t_mat) {
    box.transform(t_mat);
    {   // transform velocity
        auto tmp_dir = velocity;
        double tmp_val = tmp_dir.norm();
        tmp_dir.normalize();

        Eigen::Vector4d tt_pt;
        tt_pt << tmp_dir.head(3), 0.f;
        tt_pt = t_mat * tt_pt;

        velocity = tt_pt.head(3) * tmp_val;
    }
    {   // transform accelerate
        auto tmp_dir = accelerate;
        double tmp_val = tmp_dir.norm();
        tmp_dir.normalize();

        Eigen::Vector4d tt_pt;
        tt_pt << tmp_dir.head(3), 0.f;
        tt_pt = t_mat * tt_pt;

        accelerate = tt_pt.head(3) * tmp_val;
    }
}

}  // namespace rally
