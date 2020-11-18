#include <string.h>
#include <math.h>

#include <libplayercore/playercore.h>

#include "audioshared.h"

////////////////////////////////////////////////////////////////////////////////
// The class for the driver
class OpaqueDriver : public ThreadedDriver
{
  public:

    // Constructor; need that
    OpaqueDriver(ConfigFile* cf, int section);

    // This method will be invoked on each incoming message
    virtual int ProcessMessage(QueuePointer &resp_queue,
                               player_msghdr * hdr,
                               void * data);

  private:

    // Main function for device thread.
    virtual void Main();
    virtual int MainSetup();
    virtual void MainQuit();

    // Update the data
    virtual void RefreshData();

    // This is the structure we want to send
    our_audio_t audioStruct;

    // This is the data we store and send
    player_opaque_data_t mData;
};

// A factory creation function, declared outside of the class so that it
// can be invoked without any object context (alternatively, you can
// declare it static in the class).  In this function, we create and return
// (as a generic Driver*) a pointer to a new instance of this driver.
Driver*
OpaqueDriver_Init(ConfigFile* cf, int section)
{
  // Create and return a new instance of this driver
  return((Driver*)(new OpaqueDriver(cf, section)));
}

// A driver registration function, again declared outside of the class so
// that it can be invoked without object context.  In this function, we add
// the driver into the given driver table, indicating which interface the
// driver can support and how to create a driver instance.
void OpaqueDriver_Register(DriverTable* table)
{
  table->AddDriver("opaquedriver", OpaqueDriver_Init);
}

////////////////////////////////////////////////////////////////////////////////
// Constructor.  Retrieve options from the configuration file and do any
// pre-Setup() setup.
OpaqueDriver::OpaqueDriver(ConfigFile* cf, int section)
    : ThreadedDriver(cf, section, false, PLAYER_MSGQUEUE_DEFAULT_MAXLEN,
             PLAYER_OPAQUE_CODE)
{
  mData.data_count = sizeof(our_audio_t);
  mData.data = reinterpret_cast<uint8_t*>(&audioStruct);

	audioStruct.id = -1;
	audioStruct.sink = -1;
	audioStruct.px = 0;
	audioStruct.py = 0;

  return;
}

////////////////////////////////////////////////////////////////////////////////
// Set up the device.  Return 0 if things go well, and -1 otherwise.
int OpaqueDriver::MainSetup()
{
  puts("Example driver initialising");

  // Here you do whatever is necessary to setup the device, like open and
  // configure a serial port.

  puts("Opaque driver ready");

  return(0);
}


////////////////////////////////////////////////////////////////////////////////
// Shutdown the device
void OpaqueDriver::MainQuit()
{
  puts("Shutting opaque driver down");

  // Here you would shut the device down by, for example, closing a
  // serial port.

  puts("Opaque driver has been shutdown");
}

int OpaqueDriver::ProcessMessage(QueuePointer & resp_queue,
                                 player_msghdr* hdr,
                                 void* data)
{
  // Process messages here.  Send a response if necessary, using Publish().
  // If you handle the message successfully, return 0.  Otherwise,
  // return -1, and a NACK will be sent for you, if a response is required.
	our_audio_t *temp = (our_audio_t *)((player_opaque_data_t *)data)->data;
	audioStruct.id = temp->id;
	audioStruct.sink = temp->sink;
	audioStruct.px = temp->px;
	audioStruct.py = temp->py;
	for (int i = 0; i < 5; i++) {
		uint32_t size = sizeof(mData) - sizeof(mData.data) + mData.data_count;
		Publish(
			device_addr,
          		PLAYER_MSGTYPE_DATA, PLAYER_OPAQUE_DATA_STATE,
          		reinterpret_cast<void*>(&mData), size, NULL
		);
	}
	return(0);
}



////////////////////////////////////////////////////////////////////////////////
// Main function for device thread
void OpaqueDriver::Main()
{
  // The main loop; interact with the device here
  for(;;)
  {
    Wait(1);
    // Process incoming messages.  OpaqueDriver::ProcessMessage() is
    // called on each message.
    ProcessMessages();

    // Interact with the device, and push out the resulting data, using
    RefreshData();
  }
}

void OpaqueDriver::RefreshData()
{
	//puts("about to send");
/*	uint32_t size = sizeof(mData) - sizeof(mData.data) + mData.data_count;
		Publish(device_addr,
          PLAYER_MSGTYPE_DATA, PLAYER_OPAQUE_DATA_STATE,
          reinterpret_cast<void*>(&mData), size, NULL);*/
	//puts("full send");
}
////////////////////////////////////////////////////////////////////////////////
// Extra stuff for building a shared object.

/* need the extern to avoid C++ name-mangling  */
extern "C" {
  int player_driver_init(DriverTable* table)
  {
    puts("Opaque driver initializing");
    OpaqueDriver_Register(table);
    puts("Opaque driver done");
    return(0);
  }
}

