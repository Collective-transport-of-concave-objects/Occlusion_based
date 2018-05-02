//
//  cmd.cpp
//  
//
//  Created by Chris on 3/21/18.
//

#include "cmd.h"

    //cmd stop_cmd(STOP_CMD);
    //cmd move_cmd(MOVE_CMD);

    cmd::cmd(CMDS cc) {
        if (cc == NEED_CMD) {
            action = cc;
            X = 0.0;
            Y = 0.0;
            }
        else if (cc == MOVE_CMD) {
            action = MOVE_CMD;
            X = 1.0;
            Y = 1.0;
            }
        else if (cc == STOP_CMD) {
            action = STOP_CMD;
            X = 0.0;
            Y = 0.0;
            }
        }

    cmd::cmd() {
        cmd(MOVE_CMD);
        X = 0.0;
        Y = 1.0;
        }

    argos::Real cmd::GetX() {
        return X;
    }

    argos::Real cmd::GetY() {
        return Y;
    }

    CMDS cmd::GetAction() {
        return action;
    }

    void cmd::SetXY(float x, float y) {
        X = x;
        Y = y;
    }


