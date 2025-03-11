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

         /* Ignore events when in motion */
         fct_chk(transition(GOINGDNTO2, REQ_DOOR_OPEN) == GOINGDNTO2);  // Ignore door open while moving down
         fct_chk(transition(GOINGDNTO3, REQ_DOOR_OPEN) == GOINGDNTO3);
         fct_chk(transition(GOINGUPTO3, REQ_DOOR_OPEN) == GOINGUPTO3);
         fct_chk(transition(GOINGUPTO4, REQ_DOOR_OPEN) == GOINGUPTO4);

         /* Ignore floor requests while in motion */
         fct_chk(transition(GOINGDNTO2, CALL_FLOOR_3) == GOINGDNTO2);
         fct_chk(transition(GOINGDNTO2, CALL_FLOOR_4) == GOINGDNTO2);
         fct_chk(transition(GOINGDNTO3, CALL_FLOOR_2) == GOINGDNTO3);
         fct_chk(transition(GOINGUPTO4, CALL_FLOOR_2) == GOINGUPTO4);
         fct_chk(transition(GOINGUPTO4, CALL_FLOOR_3) == GOINGUPTO4);
         fct_chk(transition(GOINGUPTO3, CALL_FLOOR_2) == GOINGUPTO3);

         /* Ignore requests for the current floor */
         fct_chk(transition(FLOOR2, CALL_FLOOR_2) == FLOOR2);
         fct_chk(transition(FLOOR3, CALL_FLOOR_3) == FLOOR3);
         fct_chk(transition(FLOOR4, CALL_FLOOR_4) == FLOOR4);

         /* Ignore stop request if elevator is not in motion */
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
