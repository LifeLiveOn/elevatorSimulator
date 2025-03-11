// generated by Fast Light User Interface Designer (fluid) version 1.0308

#include "src/elevatorGUI.h"

void UserInterfaceElevator::cb_powerButton_i(Fl_Button*, void*) {
  if (power_status())
{
     power_off();
}
else
{
   power_on();
};
}
void UserInterfaceElevator::cb_powerButton(Fl_Button* o, void* v) {
  ((UserInterfaceElevator*)(o->parent()->parent()->user_data()))->cb_powerButton_i(o,v);
}

void UserInterfaceElevator::cb_obstruct_i(Fl_Check_Button* o, void*) {
  door_obstructed(o->value());
}
void UserInterfaceElevator::cb_obstruct(Fl_Check_Button* o, void* v) {
  ((UserInterfaceElevator*)(o->parent()->parent()->user_data()))->cb_obstruct_i(o,v);
}

void UserInterfaceElevator::cb_GUI_REQ_FLOOR_2_i(Fl_Button*, void*) {
  eventEnum event;
event=REQ_FLOOR_2;
event_to_controller(event);
}
void UserInterfaceElevator::cb_GUI_REQ_FLOOR_2(Fl_Button* o, void* v) {
  ((UserInterfaceElevator*)(o->parent()->parent()->parent()->user_data()))->cb_GUI_REQ_FLOOR_2_i(o,v);
}

void UserInterfaceElevator::cb_GUI_REQ_FLOOR_3_i(Fl_Button*, void*) {
  eventEnum event;
event=REQ_FLOOR_3;
event_to_controller(event);
}
void UserInterfaceElevator::cb_GUI_REQ_FLOOR_3(Fl_Button* o, void* v) {
  ((UserInterfaceElevator*)(o->parent()->parent()->parent()->user_data()))->cb_GUI_REQ_FLOOR_3_i(o,v);
}

void UserInterfaceElevator::cb_GUI_REQ_FLOOR_4_i(Fl_Button*, void*) {
  eventEnum event;
event=REQ_FLOOR_4;
event_to_controller(event);
}
void UserInterfaceElevator::cb_GUI_REQ_FLOOR_4(Fl_Button* o, void* v) {
  ((UserInterfaceElevator*)(o->parent()->parent()->parent()->user_data()))->cb_GUI_REQ_FLOOR_4_i(o,v);
}

void UserInterfaceElevator::cb_GUI_CALL_ACCEPTED_FLOOR_2_i(Fl_Light_Button*, void*) {
  eventEnum event;
event=REQ_FLOOR_2;
event_to_controller(event);
}
void UserInterfaceElevator::cb_GUI_CALL_ACCEPTED_FLOOR_2(Fl_Light_Button* o, void* v) {
  ((UserInterfaceElevator*)(o->parent()->parent()->user_data()))->cb_GUI_CALL_ACCEPTED_FLOOR_2_i(o,v);
}

void UserInterfaceElevator::cb_GUI_CALL_ACCEPTED_FLOOR_3_i(Fl_Light_Button*, void*) {
  eventEnum event;
event=REQ_FLOOR_3;
event_to_controller(event);
}
void UserInterfaceElevator::cb_GUI_CALL_ACCEPTED_FLOOR_3(Fl_Light_Button* o, void* v) {
  ((UserInterfaceElevator*)(o->parent()->parent()->user_data()))->cb_GUI_CALL_ACCEPTED_FLOOR_3_i(o,v);
}

void UserInterfaceElevator::cb_GUI_CALL_ACCEPTED_FLOOR_4_i(Fl_Light_Button*, void*) {
  eventEnum event;
event=REQ_FLOOR_4;
event_to_controller(event);
}
void UserInterfaceElevator::cb_GUI_CALL_ACCEPTED_FLOOR_4(Fl_Light_Button* o, void* v) {
  ((UserInterfaceElevator*)(o->parent()->parent()->user_data()))->cb_GUI_CALL_ACCEPTED_FLOOR_4_i(o,v);
}

Fl_Double_Window* UserInterfaceElevator::make_window() {
  Fl_Double_Window* w;
  { Fl_Double_Window* o = new Fl_Double_Window(1070, 375, "elevatorSimulator");
    w = o; if (w) {/* empty */}
    o->user_data((void*)(this));
    { Fl_Pack* o = new Fl_Pack(280, 25, 80, 65, "Current Floor");
      o->box(FL_BORDER_FRAME);
      { currentFloor4 = new Fl_Round_Button(285, 30, 70, 20, "4");
        currentFloor4->down_box(FL_ROUND_DOWN_BOX);
      } // Fl_Round_Button* currentFloor4
      { currentFloor3 = new Fl_Round_Button(285, 50, 70, 20, "3");
        currentFloor3->down_box(FL_ROUND_DOWN_BOX);
      } // Fl_Round_Button* currentFloor3
      { currentFloor2 = new Fl_Round_Button(285, 70, 70, 20, "2");
        currentFloor2->down_box(FL_ROUND_DOWN_BOX);
      } // Fl_Round_Button* currentFloor2
      o->end();
    } // Fl_Pack* o
    { Fl_Group* o = new Fl_Group(440, 25, 135, 125, "Power");
      { timeBox = new Fl_Output(480, 67, 70, 28, " ");
        timeBox->box(FL_FLAT_BOX);
      } // Fl_Output* timeBox
      { powerButton = new Fl_Button(475, 25, 70, 35, "null");
        powerButton->callback((Fl_Callback*)cb_powerButton);
      } // Fl_Button* powerButton
      { Fl_Check_Button* o = new Fl_Check_Button(440, 110, 70, 20, "obstruct door");
        o->down_box(FL_DOWN_BOX);
        o->callback((Fl_Callback*)cb_obstruct);
      } // Fl_Check_Button* o
      o->end();
    } // Fl_Group* o
    { Fl_Pack* o = new Fl_Pack(280, 120, 45, 240, "Cab Position");
      o->box(FL_BORDER_FRAME);
      { pos40 = new Fl_Round_Button(280, 125, 45, 20, "4");
        pos40->down_box(FL_ROUND_DOWN_BOX);
      } // Fl_Round_Button* pos40
      { pos38 = new Fl_Round_Button(280, 145, 45, 20);
        pos38->down_box(FL_ROUND_DOWN_BOX);
      } // Fl_Round_Button* pos38
      { pos36 = new Fl_Round_Button(280, 165, 45, 20);
        pos36->down_box(FL_ROUND_DOWN_BOX);
      } // Fl_Round_Button* pos36
      { pos34 = new Fl_Round_Button(280, 185, 45, 20);
        pos34->down_box(FL_ROUND_DOWN_BOX);
      } // Fl_Round_Button* pos34
      { pos32 = new Fl_Round_Button(280, 205, 45, 20);
        pos32->box(FL_BORDER_FRAME);
        pos32->down_box(FL_ROUND_DOWN_BOX);
      } // Fl_Round_Button* pos32
      { pos30 = new Fl_Round_Button(280, 225, 45, 20, "3");
        pos30->down_box(FL_ROUND_DOWN_BOX);
      } // Fl_Round_Button* pos30
      { pos28 = new Fl_Round_Button(280, 245, 45, 20);
        pos28->down_box(FL_ROUND_DOWN_BOX);
      } // Fl_Round_Button* pos28
      { pos26 = new Fl_Round_Button(280, 265, 45, 20);
        pos26->down_box(FL_ROUND_DOWN_BOX);
      } // Fl_Round_Button* pos26
      { pos24 = new Fl_Round_Button(280, 285, 45, 20);
        pos24->down_box(FL_ROUND_DOWN_BOX);
      } // Fl_Round_Button* pos24
      { pos22 = new Fl_Round_Button(280, 305, 45, 20);
        pos22->down_box(FL_ROUND_DOWN_BOX);
      } // Fl_Round_Button* pos22
      { pos20 = new Fl_Round_Button(280, 325, 45, 20, "2");
        pos20->down_box(FL_ROUND_DOWN_BOX);
      } // Fl_Round_Button* pos20
      o->end();
    } // Fl_Pack* o
    { Fl_Group* o = new Fl_Group(770, 130, 275, 460, "Elevator Cab");
      o->box(FL_BORDER_FRAME);
      { Fl_Group* o = new Fl_Group(810, 155, 185, 25, "CAB_POS_X");
        o->align(Fl_Align(FL_ALIGN_LEFT));
        { GUI_CAB_POS_2 = new Fl_Round_Button(815, 160, 70, 20, "2");
          GUI_CAB_POS_2->down_box(FL_ROUND_DOWN_BOX);
        } // Fl_Round_Button* GUI_CAB_POS_2
        { GUI_CAB_POS_3 = new Fl_Round_Button(870, 160, 70, 20, "3");
          GUI_CAB_POS_3->down_box(FL_ROUND_DOWN_BOX);
        } // Fl_Round_Button* GUI_CAB_POS_3
        { GUI_CAB_POS_4 = new Fl_Round_Button(925, 160, 70, 20, "4");
          GUI_CAB_POS_4->down_box(FL_ROUND_DOWN_BOX);
        } // Fl_Round_Button* GUI_CAB_POS_4
        o->end();
      } // Fl_Group* o
      { Fl_Group* o = new Fl_Group(800, 195, 225, 25, "CALL BUTTONS");
        o->align(Fl_Align(FL_ALIGN_LEFT));
        { GUI_REQ_FLOOR_2 = new Fl_Button(800, 197, 70, 20, "2");
          GUI_REQ_FLOOR_2->callback((Fl_Callback*)cb_GUI_REQ_FLOOR_2);
        } // Fl_Button* GUI_REQ_FLOOR_2
        { GUI_REQ_FLOOR_3 = new Fl_Button(875, 197, 70, 20, "3");
          GUI_REQ_FLOOR_3->callback((Fl_Callback*)cb_GUI_REQ_FLOOR_3);
        } // Fl_Button* GUI_REQ_FLOOR_3
        { GUI_REQ_FLOOR_4 = new Fl_Button(955, 197, 70, 20, "4");
          GUI_REQ_FLOOR_4->callback((Fl_Callback*)cb_GUI_REQ_FLOOR_4);
        } // Fl_Button* GUI_REQ_FLOOR_4
        o->end();
      } // Fl_Group* o
      { Fl_Group* o = new Fl_Group(805, 236, 225, 34, "REQ_FLOOR_ACCEPTED_X");
        o->align(Fl_Align(FL_ALIGN_LEFT));
        { GUI_REQ_FLOOR_ACCEPTED_2 = new Fl_Output(850, 241, 15, 28, "2");
        } // Fl_Output* GUI_REQ_FLOOR_ACCEPTED_2
        { GUI_REQ_FLOOR_ACCEPTED_3 = new Fl_Output(935, 242, 15, 28, "3");
        } // Fl_Output* GUI_REQ_FLOOR_ACCEPTED_3
        { GUI_REQ_FLOOR_ACCEPTED_4 = new Fl_Output(1010, 242, 15, 26, "4");
        } // Fl_Output* GUI_REQ_FLOOR_ACCEPTED_4
        o->end();
      } // Fl_Group* o
      { Fl_Group* o = new Fl_Group(795, 280, 235, 50, "Door Position");
        o->align(Fl_Align(FL_ALIGN_LEFT));
        { GUI_DOOR_POSITION = new Fl_Progress(830, 291, 130, 24, " ");
        } // Fl_Progress* GUI_DOOR_POSITION
        o->end();
      } // Fl_Group* o
      { new Fl_Button(975, 295, 70, 20, "DOOR");
      } // Fl_Button* o
      { new Fl_Light_Button(835, 335, 74, 20, "BELL");
      } // Fl_Light_Button* o
      o->end();
    } // Fl_Group* o
    { Fl_Group* o = new Fl_Group(25, 37, 145, 48, "FLOOR 2");
      { GUI_CALL_ACCEPTED_FLOOR_2 = new Fl_Light_Button(30, 51, 74, 20, "CALL");
        GUI_CALL_ACCEPTED_FLOOR_2->callback((Fl_Callback*)cb_GUI_CALL_ACCEPTED_FLOOR_2);
      } // Fl_Light_Button* GUI_CALL_ACCEPTED_FLOOR_2
      { GUI_UPPTAGEN_FLOOR_2 = new Fl_Output(155, 47, 15, 28, "BUSY");
      } // Fl_Output* GUI_UPPTAGEN_FLOOR_2
      o->end();
    } // Fl_Group* o
    { Fl_Group* o = new Fl_Group(25, 97, 145, 48, "FLOOR 3");
      { GUI_CALL_ACCEPTED_FLOOR_3 = new Fl_Light_Button(30, 111, 74, 20, "CALL");
        GUI_CALL_ACCEPTED_FLOOR_3->callback((Fl_Callback*)cb_GUI_CALL_ACCEPTED_FLOOR_3);
      } // Fl_Light_Button* GUI_CALL_ACCEPTED_FLOOR_3
      { GUI_UPPTAGEN_FLOOR_3 = new Fl_Output(155, 107, 15, 28, "BUSY");
      } // Fl_Output* GUI_UPPTAGEN_FLOOR_3
      o->end();
    } // Fl_Group* o
    { Fl_Group* o = new Fl_Group(25, 157, 145, 48, "FLOOR 4");
      { GUI_CALL_ACCEPTED_FLOOR_4 = new Fl_Light_Button(30, 171, 74, 20, "CALL");
        GUI_CALL_ACCEPTED_FLOOR_4->callback((Fl_Callback*)cb_GUI_CALL_ACCEPTED_FLOOR_4);
      } // Fl_Light_Button* GUI_CALL_ACCEPTED_FLOOR_4
      { GUI_UPPTAGEN_FLOOR_4 = new Fl_Output(155, 167, 15, 28, "BUSY");
      } // Fl_Output* GUI_UPPTAGEN_FLOOR_4
      o->end();
    } // Fl_Group* o
    o->end();
  } // Fl_Double_Window* o
  return w;
}

void UserInterfaceElevator::guiTick() {
  if (power_status()) powerButton->label("ON"); else powerButton->label("OFF"); {
  }
  // fetch the current indicators from the elevator
  unsigned int ind = indicators(); 
  (ind & CAB_POS_2) ? GUI_CAB_POS_2->value(true):GUI_CAB_POS_2->value(false);
  (ind & CAB_POS_3) ? GUI_CAB_POS_3->value(true):GUI_CAB_POS_3->value(false);
  (ind & CAB_POS_4) ? GUI_CAB_POS_4->value(true):GUI_CAB_POS_4->value(false);
  //
  (ind & REQ_FLOOR_ACCEPTED_2) ? GUI_REQ_FLOOR_ACCEPTED_2->value("+"):GUI_REQ_FLOOR_ACCEPTED_2->value("");
  (ind & REQ_FLOOR_ACCEPTED_3) ? GUI_REQ_FLOOR_ACCEPTED_3->value("+"):GUI_REQ_FLOOR_ACCEPTED_3->value("");
  (ind & REQ_FLOOR_ACCEPTED_4) ? GUI_REQ_FLOOR_ACCEPTED_4->value("+"):GUI_REQ_FLOOR_ACCEPTED_4->value("");
  //
  (ind & CALL_ACCEPTED_FLOOR_2) ? GUI_CALL_ACCEPTED_FLOOR_2->set(): GUI_CALL_ACCEPTED_FLOOR_2->clear();
  (ind & CALL_ACCEPTED_FLOOR_3) ? GUI_CALL_ACCEPTED_FLOOR_3->set(): GUI_CALL_ACCEPTED_FLOOR_3->clear();
  (ind & CALL_ACCEPTED_FLOOR_4) ? GUI_CALL_ACCEPTED_FLOOR_4->set(): GUI_CALL_ACCEPTED_FLOOR_4->clear();
  //
  (ind & UPPTAGEN_FLOOR_2) ? GUI_UPPTAGEN_FLOOR_2->value("+"):GUI_UPPTAGEN_FLOOR_2->value("");
  (ind & UPPTAGEN_FLOOR_3) ? GUI_UPPTAGEN_FLOOR_3->value("+"):GUI_UPPTAGEN_FLOOR_3->value("");
  (ind & UPPTAGEN_FLOOR_4) ? GUI_UPPTAGEN_FLOOR_4->value("+"):GUI_UPPTAGEN_FLOOR_4->value("");
  //
   GUI_DOOR_POSITION->minimum(1);
   GUI_DOOR_POSITION->maximum(5);
   GUI_DOOR_POSITION->value(door_position());
  // 
  currentFloor2->value(0);
  currentFloor3->value(0);
  currentFloor4->value(0);
  if (cab_position()==20) currentFloor2->value(1);
  if (cab_position()==30) currentFloor3->value(1);
  if (cab_position()==40) currentFloor4->value(1);
  //
  pos20->value(0);
  pos22->value(0);
  pos24->value(0);
  pos26->value(0);
  pos28->value(0);
  pos30->value(0);
  pos32->value(0);
  pos34->value(0);
  pos36->value(0);
  pos38->value(0);
  pos40->value(0);
  if (cab_position()==20) pos20->value(1);
  if (cab_position()==22) pos22->value(1);
  if (cab_position()==24) pos24->value(1);
  if (cab_position()==26) pos26->value(1);
  if (cab_position()==28) pos28->value(1);
  if (cab_position()==30) pos30->value(1);
  if (cab_position()==32) pos32->value(1);
  if (cab_position()==34) pos34->value(1);
  if (cab_position()==36) pos36->value(1);
  if (cab_position()==38) pos38->value(1);
  if (cab_position()==40) pos40->value(1);
  //
  timeBox->value(std::to_string(timeInSeconds()).c_str());
}
