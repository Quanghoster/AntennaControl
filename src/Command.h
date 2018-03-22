/*
Command.h - command objects to control antenna with
*/

#ifndef COMMAND_H
#define COMMAND_H


// base class for all command objects
class Command{
  public:
   virtual void setup()
  {}
   virtual bool loop()
  {
    return true;// ended
  }  
};



// Zenith command - causes antenna to point South at 90 degrees elevation
class Zenith : Command
{
  public:
   bool loop();
};


#endif

