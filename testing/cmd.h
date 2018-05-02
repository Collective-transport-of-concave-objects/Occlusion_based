//
//  cmd.h
//  
//
//  Created by Chris on 3/21/18.
//

#include <argos3/core/utility/configuration/argos_configuration.h>

#ifndef cmd_h
#define cmd_h

enum CMDS {NEED_CMD, STOP_CMD, MOVE_CMD};
enum STAT {READY, STOPPED, DONE, EXECUTING};

class cmd {

    private:
        CMDS action;
        argos::Real X;
        argos::Real Y;

    public:
        cmd(CMDS);
        cmd();
        argos::Real GetX();
        argos::Real GetY();
        CMDS GetAction();
        void SetXY(float, float);
};

//    cmd stop_cm;
//    cmd move_cmd;

#endif /* cmd_h */
