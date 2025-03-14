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

// each state has an "on entry" function.
// the are declared here so they can be available in the
// forthcoming data table(s).
void off_entry();
void init_entry();
void floor2_state_entry();
void floor3_state_entry();
void floor4_state_entry();
void goingdnto2_state_entry();
void goingdnto3_state_entry();
void goingupto3_state_entry();
void goingupto4_state_entry();

// array of function pointers, indexed by elevatorStateEnum
// must be in the same order as the enums are declared
void (*on_entry[GOINGDNTO2 + 1])() = {off_entry,
									  init_entry,
									  floor2_state_entry,
									  floor3_state_entry,
									  floor4_state_entry,
									  goingupto3_state_entry,
									  goingdnto3_state_entry,
									  goingupto4_state_entry,
									  goingdnto2_state_entry};

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
    /*            TIMER_EXPIRED   POWER_ON       DOOR_OPEN      DOOR_CLOSED    DOOR_OBSTRUCTED 
                  CAB_POS_2       CAB_POS_2_5    CAB_POS_3      CAB_POS_3_5    CAB_POS_4  
                  CALL_2          CALL_3         CALL_4         REQ_DOOR_OPEN  REQ_STOP  
                  REQ_FLOOR_2     REQ_FLOOR_3    REQ_FLOOR_4    REQ_BELL_PRESS REQ_BELL_RELEASE */

    /* OFF       */ {{f, INIT},    {t, INIT},    {f, OFF},      {f, OFF},      {f, OFF},  
                     {f, OFF},     {f, OFF},     {f, OFF},      {f, OFF},      {f, OFF},  
                     {f, OFF},     {f, OFF},     {f, OFF},      {f, OFF},      {f, OFF},  
                     {f, OFF},     {f, OFF},     {f, OFF},      {f, OFF},      {f, OFF}},

    /* INIT      */ {{t, FLOOR2},  {f, INIT},    {t, FLOOR2},   {t, FLOOR2},   {f, FLOOR2},  
                     {f, FLOOR2},  {f, FLOOR2},  {f, FLOOR2},   {f, FLOOR2},   {f, FLOOR2},  
                     {f, FLOOR2},  {f, GOINGUPTO3}, {f, GOINGUPTO4}, {t, FLOOR2}, {f, FLOOR2},  
                     {f, FLOOR2},  {f, GOINGUPTO3}, {f, GOINGUPTO4}, {f, FLOOR2}, {f, FLOOR2}},

    /* FLOOR2    */ {{t, FLOOR2},    {f, FLOOR2},  {t, FLOOR2},   {t, FLOOR2},   {t, FLOOR2},  
                     {t, FLOOR2},  {f, FLOOR2},  {f, FLOOR2},   {f, FLOOR2},   {f, FLOOR2},  
                     {t, FLOOR2},  {t, GOINGUPTO3}, {t, GOINGUPTO4}, {t, FLOOR2}, {f, FLOOR2},  
                     {t, FLOOR2},  {t, GOINGUPTO3}, {t, GOINGUPTO4}, {f, FLOOR2}, {f, FLOOR2}},

    /* FLOOR3    */ {{t, FLOOR3},    {f, FLOOR3},  {t, FLOOR3},   {t, FLOOR3},   {t, FLOOR3},  
                     {f, FLOOR3},  {f, FLOOR3},  {t, FLOOR3},   {f, FLOOR3},   {f, FLOOR3},  
                     {t, GOINGDNTO2}, {t, FLOOR3}, {t, GOINGUPTO4}, {t, FLOOR3}, {f, FLOOR3},  
                     {t, GOINGDNTO2}, {t, FLOOR3}, {t, GOINGUPTO4}, {f, FLOOR3}, {f, FLOOR3}},

    /* FLOOR4    */ {{t, FLOOR4},    {f, FLOOR4},  {t, FLOOR4},   {t, FLOOR4},   {t, FLOOR4},  
                     {f, FLOOR4},  {f, FLOOR4},  {f, FLOOR4},   {f, FLOOR4},   {t, FLOOR4},  
                     {t, GOINGDNTO2}, {t, GOINGDNTO3}, {t, FLOOR4}, {t, FLOOR4}, {f, FLOOR4},  
                     {t, GOINGDNTO2}, {t, GOINGDNTO3}, {t, FLOOR4}, {f, FLOOR4}, {f, FLOOR4}},

    /* GOINGUPTO3 */ {{f, INIT},   {f, GOINGDNTO3}, {f, GOINGUPTO3}, {f, GOINGUPTO3}, {f, GOINGUPTO3},  
                      {f, GOINGUPTO3}, {f, GOINGUPTO3}, {t, FLOOR3}, {f, GOINGUPTO3}, {f, GOINGUPTO3},  
                      {f, GOINGUPTO3}, {f, GOINGUPTO3}, {f, GOINGUPTO3}, {f, GOINGUPTO3}, {f, GOINGUPTO3},  
                      {f, GOINGUPTO3}, {f, GOINGUPTO3}, {f, GOINGUPTO3}, {f, GOINGUPTO3}, {f, GOINGUPTO3}},

    /* GOINGDNTO3 */ {{f, INIT},   {f, GOINGUPTO3}, {f, GOINGDNTO3}, {f, GOINGDNTO3}, {f, GOINGDNTO3},  
                      {f, GOINGDNTO3}, {f, GOINGDNTO3}, {t, FLOOR3}, {f, GOINGDNTO3}, {f, GOINGDNTO3},  
                      {f, GOINGDNTO3}, {f, GOINGDNTO3}, {f, GOINGDNTO3}, {f, GOINGDNTO3}, {f, GOINGDNTO3},  
                      {f, GOINGDNTO3}, {f, GOINGDNTO3}, {f, GOINGDNTO3}, {f, GOINGDNTO3}, {f, GOINGDNTO3}},

    /* GOINGUPTO4 */ {{f, INIT},   {f, GOINGUPTO4}, {f, GOINGUPTO4}, {f, GOINGUPTO4}, {f, GOINGUPTO4},  
                      {f, GOINGUPTO4}, {f, GOINGUPTO4}, {f, GOINGUPTO4}, {f, GOINGUPTO4}, {t, FLOOR4},  
                      {f, GOINGUPTO4}, {f, GOINGUPTO4}, {f, GOINGUPTO4}, {f, GOINGUPTO4}, {f, GOINGUPTO4},  
                      {f, GOINGUPTO4}, {f, GOINGUPTO4}, {f, GOINGUPTO4}, {f, GOINGUPTO4}, {f, GOINGUPTO4}},

    /* GOINGDNTO2 */ {{f, INIT},   {f, GOINGDNTO2}, {f, GOINGDNTO2}, {f, GOINGDNTO2}, {f, GOINGDNTO2},  
                      {t, FLOOR2}, {f, GOINGDNTO2}, {f, GOINGDNTO2}, {f, GOINGDNTO2}, {f, GOINGDNTO2},  
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

static bool door_open = false;  
static bool door_closing = false;  

bool door_is_open()
{
    return door_open;  // Return the global variable
}

static eventEnum last_event;  // Store the last received event


void event_to_controller(eventEnum e)
{
    // INFO_PRINT("Current Event: %s\n\n", eventEnumName(e));

    // Store the last event 
    last_event = e;

    // Update door state
    if (e == DOOR_IS_OPEN)
    {
        door_open = true;
        door_closing = false; 
    }
    else if (e == DOOR_IS_CLOSED)
    {
        door_open = false;
        door_closing = false; 
    }

    bool is_movement_request = (e == REQ_FLOOR_3 || e == REQ_FLOOR_4 || e == REQ_FLOOR_2 || 
                                e == CALL_FLOOR_3 || e == CALL_FLOOR_4 || e == CALL_FLOOR_2 || REQ_DOOR_OPEN);

    // NO request should be processed while door is closing or opening
    if (door_closing && is_movement_request)
    {
        // DEBUG_PRINT("Ignoring request door is closing!\n");
        return; 
    }

    // Check if the elevator is moving
    bool is_moving = (currentState == GOINGUPTO3 || currentState == GOINGUPTO4 ||
                      currentState == GOINGDNTO2 || currentState == GOINGDNTO3);

    // Ignore requests if the door is open or the elevator is moving
    if ((door_open || is_moving) && is_movement_request)
    {
        // DEBUG_PRINT("Ignoring request while door is opening or moving!\n");
        return; 
    }

    // go to new state otherwise
    currentState = transition(currentState, e);
}


// These functions are mandatory.  They must be implement with the same name and arguments
void controller_tick()
{
    if (timer)
    {
        timer--;
        if (!timer)  // When timer expires
        {
                // DEBUG_PRINT("Closing the door after delay.\n");
                elevator_control_cmd(CLOSE_DOOR);  // Close the door
				door_open = false;
				door_closing = true;
        }
		else if(door_closing){
			// INFO_PRINT("DOOR IS CLOSED!\n");
			door_closing = false;
		}
    }
}

void controller_init()
{
	DEBUG_PRINT("\n");
	currentState = OFF;
	// clear all timers
	timer = 0;
}

// The functions below are unique to the specific state diagram

void init_entry()
{
	DEBUG_PRINT("\n");
	// as part of the requirements, the door needs to be opened and all indicators turned on.
	elevator_control_cmd(OPEN_DOOR);
	elevator_indicators(-1);
	timer = 5;
}

void off_entry()
{
	// DEBUG_PRINT("Elevator turned OFF.\n");
    
    elevator_control_cmd(ALL_OFF);
    timer = 0; 

}

// handle all possible event at floor 2
void floor2_state_entry()
{
    DEBUG_PRINT("\n");
	if (!door_is_open() && (last_event == CALL_FLOOR_2 || last_event == REQ_FLOOR_2 || last_event == CAB_POSITION_FLOOR_2 || last_event == REQ_DOOR_OPEN)){
		// DEBUG_PRINT("Request FLOOR 2\n");
		elevator_control_cmd(OPEN_DOOR);
		door_open = true;
		door_closing = false; 
		timer = 7;
		if(last_event==REQ_DOOR_OPEN){
			// INFO_PRINT("Request open door in cabin\n");

		}
		else{
			elevator_indicators(CALL_ACCEPTED_FLOOR_2 | REQ_FLOOR_ACCEPTED_2 | CAB_POS_2 | POS_FLOOR_2 | UPPTAGEN_FLOOR_3 | UPPTAGEN_FLOOR_4);
		}
		
	}
    else  
    {
		// INFO_PRINT("No other request! at idle.\n");
        elevator_control_cmd(ALL_OFF);
        elevator_indicators(CAB_POS_2 | POS_FLOOR_2);
    }
}


// handle all possible event at floor 3
void floor3_state_entry()
{
	DEBUG_PRINT("\n");
	if (!door_is_open() && (last_event == CALL_FLOOR_3 || last_event == REQ_FLOOR_3 || last_event == CAB_POSITION_FLOOR_3 || last_event == REQ_DOOR_OPEN)){
		// DEBUG_PRINT("Request FLOOR 3");
		elevator_control_cmd(OPEN_DOOR);
		door_open = true;
		door_closing = false; 
		timer = 7;
		if(last_event==REQ_DOOR_OPEN){
			// INFO_PRINT("Request open door in cabin\n");

		}
		else{
			elevator_indicators(CALL_ACCEPTED_FLOOR_3 | REQ_FLOOR_ACCEPTED_3|CAB_POS_3 | POS_FLOOR_3 | UPPTAGEN_FLOOR_2 | UPPTAGEN_FLOOR_4);
		}
		
	}
    else  
    {
		// INFO_PRINT("No other request! at idle.\n");
        elevator_control_cmd(ALL_OFF);
        elevator_indicators(CAB_POS_3 | POS_FLOOR_3);
    }
}

// handle all possible event at floor 4
void floor4_state_entry()
{
	DEBUG_PRINT("\n");
	if (!door_is_open() && (last_event == CALL_FLOOR_4 || last_event == REQ_FLOOR_4 || last_event == CAB_POSITION_FLOOR_4 || last_event == REQ_DOOR_OPEN)){
		// DEBUG_PRINT("Request FLOOR 4");
		elevator_control_cmd(OPEN_DOOR);
		door_open = true;
		door_closing = false; 
		timer = 7;
		if(last_event==REQ_DOOR_OPEN){
			// INFO_PRINT("Request open door in cabin\n");
		}
		else{
			elevator_indicators(CALL_ACCEPTED_FLOOR_4 | REQ_FLOOR_ACCEPTED_4 | CAB_POS_4 | POS_FLOOR_4 | UPPTAGEN_FLOOR_3 | UPPTAGEN_FLOOR_2);
		}
	}
    else  
    {
		// INFO_PRINT("No other request! at idle.\n");
        elevator_control_cmd(ALL_OFF);
        elevator_indicators(CAB_POS_4 | POS_FLOOR_4);
    }
}

void goingdnto2_state_entry()
{
	DEBUG_PRINT("\n");
	elevator_control_cmd(GO_DOWN);
	elevator_indicators(indicators() | REQ_FLOOR_ACCEPTED_2 | CALL_ACCEPTED_FLOOR_2 | UPPTAGEN_FLOOR_3 | UPPTAGEN_FLOOR_4);
}

void goingdnto3_state_entry()
{
	DEBUG_PRINT("\n");
	elevator_control_cmd(GO_DOWN);
	elevator_indicators(indicators() | REQ_FLOOR_ACCEPTED_3 | CALL_ACCEPTED_FLOOR_3 | UPPTAGEN_FLOOR_4 | UPPTAGEN_FLOOR_2);
}

void goingupto3_state_entry()
{
	DEBUG_PRINT("\n");
	elevator_control_cmd(GO_UP);
	elevator_indicators(indicators() | REQ_FLOOR_ACCEPTED_3 | CALL_ACCEPTED_FLOOR_3 | UPPTAGEN_FLOOR_2 | UPPTAGEN_FLOOR_4);
}

void goingupto4_state_entry()
{
	DEBUG_PRINT("\n");
	elevator_control_cmd(GO_UP);
	elevator_indicators(indicators() | REQ_FLOOR_ACCEPTED_4 | CALL_ACCEPTED_FLOOR_4 | UPPTAGEN_FLOOR_2 | UPPTAGEN_FLOOR_3);
}

// This function is important for debugging and is unique for the state diagram
const char *elevatorStateEnumNames(elevatorStateEnum e)
{
	assert(e >= OFF && e <= GOINGDNTO2);
	const char *n[] = {"OFF",
					   "INIT",
					   "FLOOR2",
					   "FLOOR3",
					   "FLOOR4",
					   "GOINGUPTO3",
					   "GOINGDNTO3",
					   "GOINGUPTO4",
					   "GOINGDNTO2"};
	return n[e];
}