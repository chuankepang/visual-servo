#ifndef CTRLCOMPONENTS_H
#define CTRLCOMPONENTS_H

#include "z1_control/message/arm_common.h"
#include "z1_control/message/LowlevelCmd.h"
#include "z1_control/message/LowlevelState.h"
#include "z1_control/message/udp.h"
#include "z1_control/utilities/loop.h"
#include "z1_control/model/ArmModel.h"

namespace UNITREE_ARM {
struct CtrlComponents{
public:
    CtrlComponents();
    CtrlComponents(double deltaT, bool hasUnitreeGripper);
    ~CtrlComponents();
/*
 * Function: send udp message to z1_ctrl and receive udp message from it
 * Input:    None
 * Output:   None
 * Description: The function will call udp->send() to send datas in lowcmd to z1_ctrl 
 *              and call udp->recv() to store datas from z1_ctrl into lowstate
 */
    void sendRecv();
/*
 * Function: Set six joints commands to class lowcmd
 * Input:    q:  joint angle
 *           qd: joint velocity
 *           tau: joint (Only used in State_LOWCMD)
 * Output:   None
 */
    void armCtrl(Vec6 q, Vec6 qd, Vec6 tau);
/*
 * Function: Set gripper commands to class lowcmd
 * Input:    q:  joint angle
 *           qd: joint velocity
 *           tau: joint (Only used in State_LOWCMD)
 * Output:   None
 */
    void gripperCtrl(double gripperPos, double gripperW, double gripperTau);

    LowlevelCmd *lowcmd;
    LowlevelState *lowstate;
    double dt;// default: 0.002
    SendCmd sendCmd; // udp command to control the arm
    RecvState recvState; // the arm state receive from udp
    ArmFSMState statePast;
    ArmModel* armModel;
    UDPPort *udp;
};

}
#endif