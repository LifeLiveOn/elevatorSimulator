#include <assert.h>
#include <stdio.h>

#include "debug.h"
#include "elevatorController.h"
#include "events.h"


// This is an example elevator controller.  As part of the
// assignment, students are to re-implement the elevator controller
// to meet the requirements (state diagram) of a previous assignment.

// This controller is limited in complexity, and is provided only to
// demonstrate a working system.
//
// Specifics of this sample controller:
//       1.  On power on, the elevator is initialized to the bottom floor (floor 2),
//           doors open.
//       2.  When a 'REQUEST' button is pressed, the elevator goes to that floor.
//       3.  When a 'CALL' button is pressed, the elevator goes to that floor.
//       4.  REQUEST or CALL is acknowledged with a light.
//       5.  Once a CALL or a REQUEST is accepted, others are ignored until the
//           elevator gets to the desired floor.

static volatile elevatorStateEnum currentState;
static volatile unsigned int timer;
static eventEnum last_event;  // Store the last received event

// each state has an "on entry" function.
// the are declared here so they can be available in the
// forthcoming data table(s).
void off_entry();
void init_entry();
void idle_at_floor2_entry();
void floor2_entry();
void sensor_2_5_entry();
void idle_at_floor3_entry();
void floor3_entry();
void sensor_3_5_entry();
void idle_at_floor4_entry();
void floor4_entry();
void going_up_to3_entry();
void going_up_to4_entry();
void going_down_to3_entry();
void going_down_to2_entry();

// array of function pointers, indexed by elevatorStateEnum
// must be in the same order as the enums are declared
void (*on_entry[GOINGDNTO2 + 1])() = {
	off_entry,        
    init_entry,      
    idle_at_floor2_entry, 
    floor2_entry,     
    sensor_2_5_entry,
    idle_at_floor3_entry,  
    floor3_entry,   
    sensor_3_5_entry,
    idle_at_floor4_entry,  
    floor4_entry,    
    going_up_to3_entry,   
    going_down_to3_entry, 
    going_up_to4_entry,   
    going_down_to2_entry  };

void (*on_exit[GOINGDNTO2 + 1])() = {NULL};

typedef struct
{
	bool active; // this entry is ignored if active is false
	elevatorStateEnum nextState;
} stateInfo_t;

#define f false
#define t true
// if f, then the state is not used for anything......

const stateInfo_t fsm[GOINGDNTO2 + 1][REQ_BELL_RELEASED + 1] = {
    /*            TIMER_EXPIRED   POWER_ON       DOOR_IS_OPEN      DOOR_IS_CLOSED    DOOR_OBSTRUCTED 
                  CAB_POS_2       CAB_POS_2_5    CAB_POS_3      CAB_POS_3_5    CAB_POS_4  
                  CALL_2          CALL_3         CALL_4         REQ_DOOR_OPEN  REQ_STOP  
                  REQ_FLOOR_2     REQ_FLOOR_3    REQ_FLOOR_4    REQ_BELL_PRESS REQ_BELL_RELEASE */

    /* OFF       */ {{f, INIT},    {t, INIT},    {f, OFF},      {f, OFF},      {f, OFF},  
                     {f, OFF},     {f, OFF},     {f, OFF},      {f, OFF},      {f, OFF},  
                     {f, OFF},     {f, OFF},     {f, OFF},      {f, OFF},      {f, OFF},  
                     {f, OFF},     {f, OFF},     {f, OFF},      {f, OFF},      {f, OFF}},

    /* INIT      */ {{t, IDLE_AT_FLOOR2},  {f, IDLE_AT_FLOOR2},    {f, IDLE_AT_FLOOR2},   {f, IDLE_AT_FLOOR2},   {f, IDLE_AT_FLOOR2},  
                     {f, IDLE_AT_FLOOR2},  {f, IDLE_AT_FLOOR2},  {f, IDLE_AT_FLOOR2},   {f, IDLE_AT_FLOOR2},   {f, IDLE_AT_FLOOR2},  
                     {f, IDLE_AT_FLOOR2},  {f, IDLE_AT_FLOOR2}, {f, IDLE_AT_FLOOR2}, {f, IDLE_AT_FLOOR2}, {f, IDLE_AT_FLOOR2},  
                     {f, IDLE_AT_FLOOR2},  {f, IDLE_AT_FLOOR2}, {f, IDLE_AT_FLOOR2}, {f, IDLE_AT_FLOOR2}, {f, IDLE_AT_FLOOR2}},
	
	/* IDLE_AT_FLOOR2 */ {{f, FLOOR2},    {f, IDLE_AT_FLOOR2},  {f, FLOOR2},   {f, FLOOR2},   {f, IDLE_AT_FLOOR2},  
					 {f, SENSOR_2_5}, {f, IDLE_AT_FLOOR2}, {f, IDLE_AT_FLOOR2}, {f, IDLE_AT_FLOOR2}, {f, IDLE_AT_FLOOR2},  
					 {t, FLOOR2},  {t, GOINGUPTO3}, {t, GOINGUPTO4}, {t, FLOOR2}, {f, FLOOR2},  
					 {t, FLOOR2},  {t, GOINGUPTO3}, {t, GOINGUPTO4}, {f, FLOOR2}, {f, FLOOR2}},

    /* FLOOR2    */ {{t, IDLE_AT_FLOOR2},    {f, FLOOR2},  {f, FLOOR2},   {t, IDLE_AT_FLOOR2},   {f, FLOOR2},  
                     {f, FLOOR2},  {f, FLOOR2},  {f, FLOOR2},   {f, FLOOR2},   {f, FLOOR2},  
                     {f, FLOOR2},  {f, GOINGUPTO3}, {f, GOINGUPTO4}, {f, FLOOR2}, {f, FLOOR2},  
                     {f, FLOOR2},  {f, GOINGUPTO3}, {f, GOINGUPTO4}, {f, FLOOR2}, {f, FLOOR2}},
	
	/* SENSOR_2_5 */ {{f, FLOOR3},    {f, SENSOR_2_5},  {f, FLOOR3},   {f, FLOOR3},   {f, SENSOR_2_5},  
					 {t, FLOOR2}, {f, SENSOR_2_5}, {t, FLOOR3}, {f, SENSOR_2_5}, {f, FLOOR4},  
					 {f, FLOOR2},  {f, FLOOR3}, {f, GOINGUPTO4}, {f, FLOOR3}, {f, SENSOR_2_5},  
					 {f, FLOOR2},  {f, FLOOR3}, {f, GOINGUPTO4}, {f, SENSOR_2_5}, {f, SENSOR_2_5}},
	
	/* IDLE_AT_FLOOR3 */ {{f, FLOOR3},    {f, IDLE_AT_FLOOR3},  {f, FLOOR3},   {f, FLOOR3},   {f, IDLE_AT_FLOOR3},  
					 {f, IDLE_AT_FLOOR3}, {f, SENSOR_3_5}, {f, IDLE_AT_FLOOR3}, {f, IDLE_AT_FLOOR3}, {f, IDLE_AT_FLOOR3},  
					 {t, GOINGDNTO2}, {t, FLOOR3}, {t, GOINGUPTO4}, {t, FLOOR3}, {f, FLOOR3},  
					 {t, GOINGDNTO2}, {t, FLOOR3}, {t, GOINGUPTO4}, {f, FLOOR3}, {f, FLOOR3}},			

    /* FLOOR3    */ {{t, IDLE_AT_FLOOR3},    {f, FLOOR3},  {f, FLOOR3},   {t, IDLE_AT_FLOOR3},   {f, FLOOR3},  
                     {f, FLOOR3},  {f, FLOOR3},  {f, FLOOR3},   {f, FLOOR3},   {f, FLOOR3},  
                     {f, GOINGDNTO2}, {f, FLOOR3}, {f, GOINGUPTO4}, {f, FLOOR3}, {f, FLOOR3},  
                     {f, GOINGDNTO2}, {f, FLOOR3}, {f, GOINGUPTO4}, {f, FLOOR3}, {f, FLOOR3}},

	/* SENSOR_3_5 */ {{f, FLOOR4},    {f, SENSOR_3_5},  {f, FLOOR4},   {f, FLOOR4},   {f, SENSOR_3_5},  
					 {f, SENSOR_3_5}, {f, SENSOR_3_5}, {t, FLOOR3}, {f, FLOOR4}, {t, FLOOR4},  
					 {f, GOINGDNTO2}, {f, GOINGDNTO3}, {f, FLOOR4}, {f, FLOOR4}, {f, SENSOR_3_5},  
					 {f, GOINGDNTO2}, {f, GOINGDNTO3}, {f, FLOOR4}, {f, SENSOR_3_5}, {f, SENSOR_3_5}},

	/* IDLE_AT_FLOOR4 */ {{f, FLOOR4},    {f, IDLE_AT_FLOOR4},  {f, FLOOR4},   {f, FLOOR4},   {f, IDLE_AT_FLOOR4},  
					 {f, IDLE_AT_FLOOR4}, {f, IDLE_AT_FLOOR4}, {f, IDLE_AT_FLOOR4}, {f, IDLE_AT_FLOOR4}, {f, FLOOR4},  
					 {t, GOINGDNTO2}, {t, GOINGDNTO3}, {t, FLOOR4}, {t, FLOOR4}, {f, FLOOR4},  
					 {t, GOINGDNTO2}, {t, GOINGDNTO3}, {t, FLOOR4}, {f, FLOOR4}, {f, FLOOR4}},

    /* FLOOR4    */ {{t, IDLE_AT_FLOOR4},    {f, FLOOR4},  {f, FLOOR4},   {t, IDLE_AT_FLOOR4},   {f, FLOOR4},  
                     {f, FLOOR4},  {f, FLOOR4},  {f, FLOOR4},   {f, FLOOR4},   {f, FLOOR4},  
                     {f, GOINGDNTO2}, {f, GOINGDNTO3}, {f, FLOOR4}, {f, FLOOR4}, {f, FLOOR4},  
                     {f, GOINGDNTO2}, {f, GOINGDNTO3}, {f, FLOOR4}, {f, FLOOR4}, {f, FLOOR4}},

    /* GOINGUPTO3 */ {{f, INIT},   {f, GOINGDNTO3}, {f, GOINGUPTO3}, {f, GOINGUPTO3}, {f, GOINGUPTO3},  
                      {f, GOINGUPTO3}, {t, SENSOR_2_5}, {t, FLOOR3}, {f, GOINGUPTO3}, {f, GOINGUPTO3},  
                      {f, GOINGUPTO3}, {f, GOINGUPTO3}, {f, GOINGUPTO3}, {f, GOINGUPTO3}, {f, GOINGUPTO3},  
                      {f, GOINGUPTO3}, {f, GOINGUPTO3}, {f, GOINGUPTO3}, {f, GOINGUPTO3}, {f, GOINGUPTO3}},

    /* GOINGDNTO3 */ {{f, INIT},   {f, GOINGUPTO3}, {f, GOINGDNTO3}, {f, GOINGDNTO3}, {f, GOINGDNTO3},  
                      {f, GOINGDNTO3}, {f, GOINGDNTO3}, {t, FLOOR3}, {t, SENSOR_3_5}, {f, GOINGDNTO3},  
                      {f, GOINGDNTO3}, {f, GOINGDNTO3}, {f, GOINGDNTO3}, {f, GOINGDNTO3}, {f, GOINGDNTO3},  
                      {f, GOINGDNTO3}, {f, GOINGDNTO3}, {f, GOINGDNTO3}, {f, GOINGDNTO3}, {f, GOINGDNTO3}},

    /* GOINGUPTO4 */ {{f, INIT},   {f, GOINGUPTO4}, {f, GOINGUPTO4}, {f, GOINGUPTO4}, {f, GOINGUPTO4},  
                      {f, GOINGUPTO4}, {f, GOINGUPTO4}, {f, GOINGUPTO4}, {t, SENSOR_3_5}, {t, FLOOR4},  
                      {f, GOINGUPTO4}, {f, GOINGUPTO4}, {f, GOINGUPTO4}, {f, GOINGUPTO4}, {f, GOINGUPTO4},  
                      {f, GOINGUPTO4}, {f, GOINGUPTO4}, {f, GOINGUPTO4}, {f, GOINGUPTO4}, {f, GOINGUPTO4}},

    /* GOINGDNTO2 */ {{f, INIT},   {f, GOINGDNTO2}, {f, GOINGDNTO2}, {f, GOINGDNTO2}, {f, GOINGDNTO2},  
                      {t, FLOOR2}, {t, SENSOR_2_5}, {f, GOINGDNTO2}, {f, GOINGDNTO2}, {f, GOINGDNTO2},  
                      {f, GOINGDNTO2}, {f, GOINGDNTO2}, {f, GOINGDNTO2}, {f, GOINGDNTO2}, {f, GOINGDNTO2},  
                      {f, GOINGDNTO2}, {f, GOINGDNTO2}, {f, GOINGDNTO2}, {f, GOINGDNTO2}, {f, GOINGDNTO2}}
};


// after formatting the table, perhaps reading in a file would be easier.
// but, you can't beat looking at code compared to debugging it.

elevatorStateEnum transition(elevatorStateEnum state, eventEnum event)
{
	assert(state >= OFF && state <= GOINGDNTO2);
	assert(event >= TIMER_EXPIRED && event <= REQ_BELL_RELEASED);

	elevatorStateEnum nextState = state;

	// run the "on entry" for the new state
	if (fsm[state][event].active)
	{
		INFO_PRINT("current state = %s\n", elevatorStateEnumNames(state));

		// run the exit actions
		if (on_exit[state])
		{
			assert(on_entry[state]);
			(on_exit[state])();
		}

		// determine next state.
		nextState = fsm[state][event].nextState;
		assert(nextState >= OFF && nextState <= GOINGDNTO2);

		INFO_PRINT("new state = %s\n", elevatorStateEnumNames(nextState));

		// run the entry actions
		if (on_entry[nextState])
		{
			assert(on_entry[nextState]);
			(on_entry[nextState])();
		}
	}

	return nextState;
}

void event_to_controller(eventEnum e) {
    INFO_PRINT("event to controller %s\n", eventEnumName(e));
    last_event = TIMER_EXPIRED;
    if (e== DOOR_IS_CLOSED && last_event == TIMER_EXPIRED){
        event_to_controller(TIMER_EXPIRED);
        return;
    }
    last_event = e;
    currentState = transition(currentState, e);

}


void controller_tick() {
    if (timer) {
        timer--;
        if (!timer) {
            elevator_control_cmd(CLOSE_DOOR);
            return;
           
        }
    }
}

void controller_init() {
    currentState = OFF;
    timer = 0;
}

void off_entry() {
    elevator_control_cmd(ALL_OFF);
    timer = 0;
}

void init_entry() {
    elevator_control_cmd(OPEN_DOOR);
    elevator_indicators(-1);
    timer = 5;
}

void idle_at_floor2_entry() {
    elevator_control_cmd(CLOSE_DOOR);
    elevator_indicators(POS_FLOOR_2 | CAB_POS_2);
}

void floor2_entry() {
    elevator_control_cmd(OPEN_DOOR);
    timer = 5;
    elevator_indicators(POS_FLOOR_2 | CAB_POS_2 | UPPTAGEN_FLOOR_2 | REQ_FLOOR_ACCEPTED_2 | CALL_ACCEPTED_FLOOR_2);
}

void sensor_2_5_entry() {
    // elevator_indicators(CAB_POS_2_5);
}

void idle_at_floor3_entry() {
    elevator_control_cmd(CLOSE_DOOR);
    elevator_indicators(POS_FLOOR_3 | CAB_POS_3);
}

void floor3_entry() {
    elevator_control_cmd(OPEN_DOOR);
    timer = 5;
    elevator_indicators(POS_FLOOR_3 | CAB_POS_3 | UPPTAGEN_FLOOR_3 | REQ_FLOOR_ACCEPTED_3 | CALL_ACCEPTED_FLOOR_3);
}

void sensor_3_5_entry() {
    // elevator_indicators(CAB_POS_3_5);
}

void idle_at_floor4_entry() {
    elevator_control_cmd(CLOSE_DOOR);
    elevator_indicators(POS_FLOOR_4 | CAB_POS_4);
}

void floor4_entry() {
    elevator_control_cmd(OPEN_DOOR);
    timer = 5;
    elevator_indicators(POS_FLOOR_4 | CAB_POS_4 | UPPTAGEN_FLOOR_4 | REQ_FLOOR_ACCEPTED_4 | CALL_ACCEPTED_FLOOR_4);
}

void going_up_to3_entry() {
    elevator_control_cmd(GO_UP);
    elevator_indicators(UPPTAGEN_FLOOR_3 | REQ_FLOOR_ACCEPTED_3 | CALL_ACCEPTED_FLOOR_3);
}

void going_up_to4_entry() {
    elevator_control_cmd(GO_UP);
    elevator_indicators(UPPTAGEN_FLOOR_4 | REQ_FLOOR_ACCEPTED_4 | CALL_ACCEPTED_FLOOR_4);
}

void going_down_to3_entry() {
    elevator_control_cmd(GO_DOWN);
    elevator_indicators(UPPTAGEN_FLOOR_3 | REQ_FLOOR_ACCEPTED_3 | CALL_ACCEPTED_FLOOR_3);
}

void going_down_to2_entry() {
    elevator_control_cmd(GO_DOWN);
    elevator_indicators(UPPTAGEN_FLOOR_2| REQ_FLOOR_ACCEPTED_2 | CALL_ACCEPTED_FLOOR_2);
}

// This function is important for debugging and is unique for the state diagram
const char *elevatorStateEnumNames(elevatorStateEnum e) {
    assert(e >= OFF && e <= GOINGDNTO2);
    const char *n[] = {"OFF",
                       "INIT",
                       "IDLE_AT_FLOOR2",
                       "FLOOR2",
                       "SENSOR_2_5",
                       "IDLE_AT_FLOOR3",
                       "FLOOR3",
                       "SENSOR_3_5",
                       "IDLE_AT_FLOOR4",
                       "FLOOR4",
                       "GOINGUPTO3",
                       "GOINGDNTO3",
                       "GOINGUPTO4",
                       "GOINGDNTO2"};
    return n[e];
}