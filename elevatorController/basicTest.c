#include "elevatorController.h"

// https://github.com/imb/fctx/tree/master

#include "fct.h"
#include <string.h>

FCT_BGN()
{
    
   FCT_SUITE_BGN(elevator controller unit tests)
   {

      FCT_TEST_BGN(fsm transition)
      {
         printf("\n");
         /*
            first, check the most basic thing.  is the fsm table done correctly?
            lets pick a few transitions to test.

         

            these tests will break if the table changes....not ideal, but the way it is.
         */
         /* these are valid transitions */ 
         fct_chk(transition(OFF, POWER_ON) == INIT);

         fct_chk(transition(GOINGDNTO2, CAB_POSITION_FLOOR_2) == FLOOR2);
         fct_chk(transition(GOINGDNTO3, CAB_POSITION_FLOOR_3) == FLOOR3);
         fct_chk(transition(GOINGUPTO3, CAB_POSITION_FLOOR_3) == FLOOR3);
         fct_chk(transition(GOINGUPTO4, CAB_POSITION_FLOOR_4) == FLOOR4);

         fct_chk(transition(FLOOR2, CALL_FLOOR_3) == GOINGUPTO3);
         fct_chk(transition(FLOOR2, CALL_FLOOR_4) == GOINGUPTO4);
         fct_chk(transition(FLOOR3, CALL_FLOOR_2) == GOINGDNTO2);
         fct_chk(transition(FLOOR3, CALL_FLOOR_4) == GOINGUPTO4);
         fct_chk(transition(FLOOR4, CALL_FLOOR_2) == GOINGDNTO2);
         fct_chk(transition(FLOOR4, CALL_FLOOR_3) == GOINGDNTO3);

         fct_chk(transition(FLOOR2, REQ_FLOOR_3) == GOINGUPTO3);
         fct_chk(transition(FLOOR2, REQ_FLOOR_4) == GOINGUPTO4);
         fct_chk(transition(FLOOR3, REQ_FLOOR_2) == GOINGDNTO2);
         fct_chk(transition(FLOOR3, REQ_FLOOR_4) == GOINGUPTO4);
         fct_chk(transition(FLOOR4, REQ_FLOOR_2) == GOINGDNTO2);
         fct_chk(transition(FLOOR4, REQ_FLOOR_3) == GOINGDNTO3);
         printf("\n");
      }   
      FCT_TEST_END();
      FCT_TEST_BGN(door_test)
      {
         printf("\n");
         /* should ignore door open when elevator is moving */
         fct_chk(transition(GOINGDNTO2, REQ_DOOR_OPEN) == GOINGDNTO2);  
         fct_chk(transition(GOINGDNTO3, REQ_DOOR_OPEN) == GOINGDNTO3);
         fct_chk(transition(GOINGUPTO3, REQ_DOOR_OPEN) == GOINGUPTO3);
         fct_chk(transition(GOINGUPTO4, REQ_DOOR_OPEN) == GOINGUPTO4);
         
         /* Check door closed at the right state */
         fct_chk(transition(FLOOR2, DOOR_IS_CLOSED) == FLOOR2);
         fct_chk(transition(FLOOR3, DOOR_IS_CLOSED) == FLOOR3);
         fct_chk(transition(FLOOR4, DOOR_IS_CLOSED) == FLOOR4);
         /* Check door closed when timer EXPIRED */
         fct_chk(transition(FLOOR2, TIMER_EXPIRED) == FLOOR2);
         fct_chk(transition(FLOOR3, TIMER_EXPIRED) == FLOOR3);
         fct_chk(transition(FLOOR4, TIMER_EXPIRED) == FLOOR4);

         /* Check door open when requested */
         fct_chk(transition(FLOOR2, REQ_DOOR_OPEN) == FLOOR2);
         fct_chk(transition(FLOOR3, REQ_DOOR_OPEN) == FLOOR3);
         fct_chk(transition(FLOOR4, REQ_DOOR_OPEN) == FLOOR4);
      }
      FCT_TEST_END();
      FCT_TEST_BGN(sensor_test)
      {
         /* check sensors */
         fct_chk(transition(GOINGUPTO3, CAB_POSITION_FLOOR_3) == FLOOR3);
         fct_chk(transition(GOINGDNTO2, CAB_POSITION_FLOOR_2) == FLOOR2);
         fct_chk(transition(GOINGUPTO4, CAB_POSITION_FLOOR_4) == FLOOR4);

         /* should ignore mid-floor sensor check */
         fct_chk(transition(GOINGUPTO3, CAB_POSITION_FLOOR_3_5) == GOINGUPTO3);
         fct_chk(transition(GOINGDNTO3, CAB_POSITION_FLOOR_3_5) == GOINGDNTO3);
         printf("\n");
      }
      FCT_TEST_END();

      FCT_TEST_BGN(motion_test)
      {
         printf("\n");
         /* should ignore requests while elevator is moving */
         fct_chk(transition(GOINGDNTO2, CALL_FLOOR_3) == GOINGDNTO2);
         fct_chk(transition(GOINGDNTO2, CALL_FLOOR_4) == GOINGDNTO2);
         fct_chk(transition(GOINGDNTO3, CALL_FLOOR_2) == GOINGDNTO3);
         fct_chk(transition(GOINGUPTO4, CALL_FLOOR_2) == GOINGUPTO4);
         fct_chk(transition(GOINGUPTO4, CALL_FLOOR_3) == GOINGUPTO4);
         fct_chk(transition(GOINGUPTO3, CALL_FLOOR_2) == GOINGUPTO3);

         /* check if door open at current floor when pressed */
         fct_chk(transition(FLOOR2, CALL_FLOOR_2) == FLOOR2);
         fct_chk(transition(FLOOR3, CALL_FLOOR_3) == FLOOR3);
         fct_chk(transition(FLOOR4, CALL_FLOOR_4) == FLOOR4);

         /* should ignore stop request because it not implemented... */
         fct_chk(transition(FLOOR2, REQ_STOP) == FLOOR2);
         fct_chk(transition(FLOOR3, REQ_STOP) == FLOOR3);
         fct_chk(transition(FLOOR4, REQ_STOP) == FLOOR4);

         printf("\n");
      }
      FCT_TEST_END();

      FCT_TEST_BGN(chk_neq)
      {
         fct_chk(strcmp("daka", "durka") != 0);
      }
      FCT_TEST_END();

      /* Every test suite must be closed. */
   }
   FCT_SUITE_END();

   /* Every FCT scope has an end. */
}
FCT_END();
