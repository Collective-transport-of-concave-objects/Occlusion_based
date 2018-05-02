/* Include the controller definition */
#include "collective_transport.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/logging/argos_log.h>
#include<math.h>

#define PI 3.14159

CKheperaIVDiffusion::CKheperaIVDiffusion() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(2.5f),
   m_fRotVelocity(0.1f),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)),
   m_pcPosition(NULL),
   m_pcLight(NULL),
   command(MOVE_CMD),
   status(READY) {}

   void CKheperaIVDiffusion::Init(TConfigurationNode& t_node) {

   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_KheperaIVProximitySensor    >("kheperaiv_proximity"  );
   m_pcPosition  = GetSensor  <CCI_PositioningSensor           >("positioning"          );
   m_pcLight     = GetSensor  <CCI_KheperaIVLightSensor          >("kheperaiv_light"        );
    m_sPosition = m_pcPosition->GetReading().Position;

    float my_x =m_sPosition.GetX();
    float my_y =m_sPosition.GetY();
   GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
   }

   void CKheperaIVDiffusion::setBoxLocation(float x, float y){

    boxXY[0] = x;
    boxXY[1] = y;
}

float CKheperaIVDiffusion::getBoxX(){
    return boxXY[0];
}

float CKheperaIVDiffusion::getBoxY(){
    return boxXY[1];
}
bool CKheperaIVDiffusion::inRange(float currOrientation, float desiredOrientation){

    if (currOrientation < desiredOrientation+0.1 && currOrientation > desiredOrientation-0.1)
        return true;
    return false;
}
bool CKheperaIVDiffusion::inObjectRange(float getBoxX(), float getBoxY(),float my_x,float my_y){


    if ((getBoxX()< my_x+0.5 || getBoxX() > my_x-0.5) && (getBoxY()< my_y+0.5 || getBoxY()>my_y-0.5))
     
        return true;
    return false;
    
}

// bool CKheperaIVDiffusion::Light(tLightReads){

//     float v = 0.0;
//     for(size_t i=0;i<tLightReads.size();++i)
//     {
//         v=v+tLightReads[i].Value;  
//     } 

//     if ()

//         return true;
//     return false;
// }

bool CKheperaIVDiffusion::Light() {
   /* Get readings from light sensor */
   const CCI_KheperaIVLightSensor::TReadings& tLightReads = m_pcLight->GetReadings();
   /* Sum them together */
   CVector2 cAccumulator;
   for(size_t i = 0; i < tLightReads.size(); ++i) {
      cAccumulator += CVector2(tLightReads[i].Value, tLightReads[i].Angle);
   }
   /* If the light was perceived, return the vector */
   if(cAccumulator.Length() > 0.0f) {
      return true;
   }
   /* Otherwise, return zero */
   else {
      return false;
   }
}



void CKheperaIVDiffusion::ControlStep() {


    CRadians vx, vy, vz;

    loop_count++;
    if (loop_count == 5) {
        loop_count = 0;
        }

    CMDS t = command.GetAction();
    const CCI_KheperaIVProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
    const CCI_KheperaIVLightSensor::TReadings& tLightReads = m_pcLight->GetReadings();


    switch (t) {
         case NEED_CMD: {
            m_pcWheels->SetLinearVelocity(0.0, 0.0);
            break;
            }
        case STOP_CMD: {
            m_pcWheels->SetLinearVelocity(0.0, 0.0);
            status = STOPPED;
            break;
            }

            case MOVE_CMD: {



            status = EXECUTING;
            m_sPosition = m_pcPosition->GetReading().Position;

            float my_x =m_sPosition.GetX();
            float my_y =m_sPosition.GetY();
            //RLOG<<"My Position : "<<m_sPosition<<std::endl;
            RLOG<<"Robot X : "<<my_x<<std::endl;
            RLOG<<"Robot Y : "<<my_y<<std::endl;
            //TReadings& Prox =m_pcProximity->GetReadings();
            
            //RLOG<<" Prox : "<<tProxReads<<std::endl;
            //RLOG<<" Light : "<<tLightReads<<std::endl;
            // float X_sq =pow((getBoxX()-my_x),2);
            // float Y_sq =pow(getBoxY()-my_y,2);
            // float dist= sqrtf(X_sq + Y_sq);


            CQuaternion q = m_pcPosition->GetReading().Orientation;
            //RLOG<<"Orientation : "<<q<<std::endl;
            CRadians cZAngle, cYAngle, cXAngle; 
            q.ToEulerAngles(cZAngle, cYAngle, cXAngle); 

            CDegrees zAngle = ToDegrees(cZAngle);
            float theta = zAngle.GetValue();

            float orientation = atan2((getBoxY()-my_y),(getBoxX()-my_x))*180/PI;

        const CCI_KheperaIVProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
            int rotSign;
            if (orientation-theta<0)
                rotSign = 1;
            else
                rotSign = -1;

            if(oriented==false)
            {

               // RLOG<<"CURR ORIENTATION : "<<theta<<std::endl;
               // RLOG<<"DESIRED ORIENTATION : "<<orientation<<std::endl;

                m_pcWheels->SetLinearVelocity(rotSign*m_fRotVelocity, -rotSign*m_fRotVelocity);
                
                if(inRange(theta, orientation))
                {
                    m_pcWheels->SetLinearVelocity(0, 0);
                    oriented = true;
                    //RLOG<<"I am Oriented"<<std::endl;
                }
            }
        
            if(oriented ==true)
            { 
                if(!((Abs(getBoxX() - my_x) < 0.1) && (Abs(getBoxY() - my_y) < 0.1)))
                {

                    //float correction = (sqrt(pow((boxXY[0]-currXY[0]),2) + pow((boxXY[1]-currXY[1]),2)) - 20)*10;
                     //RLOG << "Difference: " << (CVector3(getBoxX(), getBoxY(), 0)/*-destination*/) << std::endl;

                    m_pcWheels->SetLinearVelocity(m_fWheelVelocity/* - correction*/, m_fWheelVelocity/* - correction*/);
                }  
                else 
                {
                    oriented = false;
                    status = DONE;
                    break;
                }
            }
            if (((getBoxX() < my_x+0.4) || (getBoxX() > my_x-0.4)) && ((getBoxY()< my_y+0.4) || (getBoxY()>my_y-0.4)))
            {   
                CVector2 cAccumulator1;
                for(size_t i = 0; i < tProxReads.size(); ++i)
                {
                    cAccumulator1 += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
                }
                    cAccumulator1 /= tProxReads.size();
                /* If the angle of the vector is small enough and the closest obstacle
                 * is far enough, continue going straight, otherwise curve a little
                */      
                float lws = 20.0;
                float rws = 20.0;
            
                CRadians cAngle = cAccumulator1.Angle();
                ////////////////////////////////////////////////////////// LEFT HAND WALL FOLLOWING ////////////////////////////////
                int k =0;
                for(size_t j = 0; j < 8; ++j)
                {

                    if(tProxReads[j].Value > 0)
                {
                    k=k+1;
                }
                
                }
                if((k<5) && (k>=1))
        {
                    RLOG<<"Length = "<<cAccumulator1.Length()<<std::endl;
                    if ((k == 3) && (tProxReads[0].Value == 0.0) && (tProxReads[4].Value == 0.0))
                {
                
                    m_pcWheels->SetLinearVelocity(lws*0.2,rws*0.2);
                }
                if(k == 4)
                {
                    m_pcWheels->SetLinearVelocity(lws*0.0,rws*0.1);
                }
            // For Angle zero 
                if((cAccumulator1.Length() < 0.07) && (cAngle.GetValue() == 0.0f ))
                {
                m_pcWheels->SetLinearVelocity(lws*0.2,rws*0.2);
              //RLOG << "Distance=  " <<dist<<std::endl;
                 }
            if((cAccumulator1.Length() < 0.09))
            {
                m_pcWheels->SetLinearVelocity(lws*0.9,rws*0.0);
              //RLOG << "Distance=  " <<dist<<std::endl;
            }
            if((cAccumulator1.Length() < 0.04))
            {
                m_pcWheels->SetLinearVelocity(lws,rws*0.0);
              //RLOG << "Distance=  " <<dist<<std::endl;
            }
            // For 0 to 90
            if((cAccumulator1.Length() > 0.03) && (cAngle.GetValue() > -1.57) && (cAngle.GetValue()<0.1))
            {
                m_pcWheels->SetLinearVelocity(0.0,rws*0.3);
              //RLOG << "Distance=  " <<dist<<std::endl;
            }
            if((cAccumulator1.Length() > 0.03) && (cAngle.GetValue() > -1.8 ) && (cAngle.GetValue()<-1.40))
            {
                m_pcWheels->SetLinearVelocity(lws*0.3,rws*0.3);
              //RLOG << "Distance=  " <<dist<<std::endl;
            }
            if((cAccumulator1.Length() > 0.03) && (cAngle.GetValue() > -3.15 ) && (cAngle.GetValue()<-1.90))
            {
                m_pcWheels->SetLinearVelocity(lws*0.5,rws*0.1);
              //RLOG << "Distance=  " <<dist<<std::endl;
            }
            if((cAccumulator1.Length() > 0.03) && (cAngle.GetValue() > -0.1 ) && (cAngle.GetValue()<3.1))
            {
                m_pcWheels->SetLinearVelocity(0.0,rws*0.5);
              //RLOG << "Distance=  " <<dist<<std::endl;
            }
            // Back right 
        }
        }

              
            //  if ((tProxReads[7].Value > 0.6) && (tProxReads[0].Value > 0.6) && (tProxReads[1].Value > 0.6))
            // {
            //     m_pcWheels->SetLinearVelocity(0.0,20.0);

            // }
            // if (tProxReads[7].Value > 0.8)
            // {
            //     m_pcWheels->SetLinearVelocity(0.0,20.0);
            // }
            // if (tProxReads[6].Value > 0.8)
            // {
            //     m_pcWheels->SetLinearVelocity(20.0,20.0);
            //     if (tProxReads[6].Value < 0.5)
            // {
            //     m_pcWheels->SetLinearVelocity(20.0,0.0);
            // }
            // }
            // if (tProxReads[5].Value > 0)
            // {
            //     m_pcWheels->SetLinearVelocity(20.0,0.0);
            //     if (tProxReads[6].Value < 0.8)
            // {
            //     m_pcWheels->SetLinearVelocity(20.0,0.0);
            // }
            // if (tProxReads[6].Value < 0.5)
            // {
            //     m_pcWheels->SetLinearVelocity(20.0,0.0);
            // }
        
            //RLOG<<"Goal Occluded"<<std::endl;
            //m_pcWheels->SetLinearVelocity(0.0,0.0);
        //     int rotSign;
        //     if (orientation-theta<0)
        //         rotSign = 1;
        //     else
        //         rotSign = -1;

        //     if(oriented==false)
        //     {

        //        // RLOG<<"CURR ORIENTATION : "<<theta<<std::endl;
        //        // RLOG<<"DESIRED ORIENTATION : "<<orientation<<std::endl;

        //         m_pcWheels->SetLinearVelocity(rotSign*m_fRotVelocity, -rotSign*m_fRotVelocity);
                
        //         if(inRange(theta, orientation))
        //         {
        //             m_pcWheels->SetLinearVelocity(0, 0);
        //             oriented = true;
        //             //RLOG<<"I am Oriented"<<std::endl;
        //         }
        //     }
        
        //     if(oriented ==true)
        //     { 
        //         if(!((Abs(getBoxX() - my_x) < 0.1) && (Abs(getBoxY() - my_y) < 0.1)))
        //         {

        //             //float correction = (sqrt(pow((boxXY[0]-currXY[0]),2) + pow((boxXY[1]-currXY[1]),2)) - 20)*10;
        //              //RLOG << "Difference: " << (CVector3(getBoxX(), getBoxY(), 0)/*-destination*/) << std::endl;

        //             m_pcWheels->SetLinearVelocity(m_fWheelVelocity/* - correction*/, m_fWheelVelocity/* - correction*/);
        //         }  
        //         else 
        //         {
        //             oriented = false;
        //             status = DONE;
        //             break;
        //         }
        //     }
            
        

        // }
        // } 
        // // default: {
        //     LOG << "Unknown command received" << std::endl;
           
        //     }
        //     break;
        if (Light() == false)
        {
                CVector2 cAccumulator1;
                for(size_t i = 0; i < tProxReads.size(); ++i)
                {
                    cAccumulator1 += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
                }
                    cAccumulator1 /= tProxReads.size();
                /* If the angle of the vector is small enough and the closest obstacle
                 * is far enough, continue going straight, otherwise curve a little
                */      
                float lws = 20.0;
                float rws = 20.0;
            
                CRadians cAngle = cAccumulator1.Angle();
            m_pcWheels->SetLinearVelocity(lws*0.5,0.0);
            if((cAngle.GetValue() > -0.2) && (cAngle.GetValue() < 0.2))
            {
                m_pcWheels->SetLinearVelocity(lws*0.5,rws*0.5);
            }
        }
    }
}
}


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //}



    void CKheperaIVDiffusion::SetCommand(cmd c) {

    }

    REGISTER_CONTROLLER(CKheperaIVDiffusion, "collective_transport_controller")
