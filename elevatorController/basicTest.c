#include "elevatorController.h"
#include "fct.h"
#include <string.h>
#include <stdio.h>

FCT_BGN() {
    FCT_SUITE_BGN(elevator controller unit tests) {

        FCT_TEST_BGN(fsm transition) {
            printf("\n");
            /*
                Validate FSM table transitions.
            */
            /* Basic power-on transition */
            fct_chk(transition(OFF, POWER_ON) == INIT);

            /* Valid state transitions */
            fct_chk(transition(GOINGDNTO2, CAB_POSITION_FLOOR_2) == FLOOR2);
            fct_chk(transition(GOINGDNTO3, CAB_POSITION_FLOOR_3) == FLOOR3);
            fct_chk(transition(GOINGUPTO3, CAB_POSITION_FLOOR_3) == FLOOR3);
            fct_chk(transition(GOINGUPTO4, CAB_POSITION_FLOOR_4) == FLOOR4);

            /* Floor call & request tests */
            fct_chk(transition(IDLE_AT_FLOOR2, CALL_FLOOR_3) == GOINGUPTO3);
            fct_chk(transition(IDLE_AT_FLOOR2, CALL_FLOOR_4) == GOINGUPTO4);
            fct_chk(transition(IDLE_AT_FLOOR3, CALL_FLOOR_2) == GOINGDNTO2);
            fct_chk(transition(IDLE_AT_FLOOR3, CALL_FLOOR_4) == GOINGUPTO4);
            fct_chk(transition(IDLE_AT_FLOOR4, CALL_FLOOR_2) == GOINGDNTO2);
            fct_chk(transition(IDLE_AT_FLOOR4, CALL_FLOOR_3) == GOINGDNTO3);

            /* Request-based movement */
            fct_chk(transition(IDLE_AT_FLOOR2, REQ_FLOOR_3) == GOINGUPTO3);
            fct_chk(transition(IDLE_AT_FLOOR2, REQ_FLOOR_4) == GOINGUPTO4);
            fct_chk(transition(IDLE_AT_FLOOR3, REQ_FLOOR_2) == GOINGDNTO2);
            fct_chk(transition(IDLE_AT_FLOOR3, REQ_FLOOR_4) == GOINGUPTO4);
            fct_chk(transition(IDLE_AT_FLOOR4, REQ_FLOOR_2) == GOINGDNTO2);
            fct_chk(transition(IDLE_AT_FLOOR4, REQ_FLOOR_3) == GOINGDNTO3);
            printf("\n");
        }
        FCT_TEST_END();

        FCT_TEST_BGN(door_test) {
            printf("\n");

            /* Doors should not open while moving */
            fct_chk(transition(GOINGDNTO2, REQ_DOOR_OPEN) == GOINGDNTO2);
            fct_chk(transition(GOINGDNTO3, REQ_DOOR_OPEN) == GOINGDNTO3);
            fct_chk(transition(GOINGUPTO3, REQ_DOOR_OPEN) == GOINGUPTO3);
            fct_chk(transition(GOINGUPTO4, REQ_DOOR_OPEN) == GOINGUPTO4);

            /* Check DOOR_IS_CLOSED event handling */
            fct_chk(transition(FLOOR2, DOOR_IS_CLOSED) == IDLE_AT_FLOOR2);
            fct_chk(transition(FLOOR3, DOOR_IS_CLOSED) == IDLE_AT_FLOOR3);
            fct_chk(transition(FLOOR4, DOOR_IS_CLOSED) == IDLE_AT_FLOOR4);

            /* Ensure door remains open if requested */
            fct_chk(transition(IDLE_AT_FLOOR2, REQ_DOOR_OPEN) == FLOOR2);
            fct_chk(transition(IDLE_AT_FLOOR3, REQ_DOOR_OPEN) == FLOOR3);
            fct_chk(transition(IDLE_AT_FLOOR4, REQ_DOOR_OPEN) == FLOOR4);
        }
        FCT_TEST_END();

        FCT_TEST_BGN(sensor_test) {
            /* Validate floor sensor triggers */
            fct_chk(transition(GOINGUPTO3, CAB_POSITION_FLOOR_3) == FLOOR3);
            fct_chk(transition(GOINGDNTO2, CAB_POSITION_FLOOR_2) == FLOOR2);
            fct_chk(transition(GOINGUPTO4, CAB_POSITION_FLOOR_4) == FLOOR4);

            /* Ignore mid-floor sensors */
            fct_chk(transition(GOINGUPTO3, CAB_POSITION_FLOOR_2_5) == SENSOR_2_5);
            fct_chk(transition(GOINGDNTO3, CAB_POSITION_FLOOR_3_5) == SENSOR_3_5);
            printf("\n");
        }
        FCT_TEST_END();

        FCT_TEST_BGN(motion_test) {
            printf("\n");

            /* Ignore movement requests while in transit */
            fct_chk(transition(GOINGDNTO2, CALL_FLOOR_3) == GOINGDNTO2);
            fct_chk(transition(GOINGDNTO2, CALL_FLOOR_4) == GOINGDNTO2);
            fct_chk(transition(GOINGDNTO3, CALL_FLOOR_2) == GOINGDNTO3);
            fct_chk(transition(GOINGUPTO4, CALL_FLOOR_2) == GOINGUPTO4);
            fct_chk(transition(GOINGUPTO4, CALL_FLOOR_3) == GOINGUPTO4);
            fct_chk(transition(GOINGUPTO3, CALL_FLOOR_2) == GOINGUPTO3);

            /* Open door when button is pressed at current floor */
            fct_chk(transition(IDLE_AT_FLOOR2, CALL_FLOOR_2) == FLOOR2);
            fct_chk(transition(IDLE_AT_FLOOR3, CALL_FLOOR_3) == FLOOR3);
            fct_chk(transition(IDLE_AT_FLOOR4, CALL_FLOOR_4) == FLOOR4);

            /* Stop request should be ignored (not implemented) */
            fct_chk(transition(IDLE_AT_FLOOR2, REQ_STOP) == IDLE_AT_FLOOR2);
            fct_chk(transition(IDLE_AT_FLOOR3, REQ_STOP) == IDLE_AT_FLOOR3);
            fct_chk(transition(IDLE_AT_FLOOR4, REQ_STOP) == IDLE_AT_FLOOR4);

            printf("\n");
        }
        FCT_TEST_END();

        FCT_TEST_BGN(chk_neq) {
            fct_chk(strcmp("testA", "testB") != 0);
        }
        FCT_TEST_END();

    }
    FCT_SUITE_END();
}
FCT_END();
