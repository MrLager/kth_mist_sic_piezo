# MIST Space Protocol (MSP) library
The MSP library consists of an experiment implementation of MSP and an OBC
implementation of MSP, both written in ANSI C. Both implementations will also
compile for C++ targets.

I2C drivers are already provided for Arduino Uno and Arduino Due platforms.

For use case examples of how the MSP library can be integrated into your code,
consult the examples under the `examples/` directory.

## Requirements
To use the provided configuration script, you need to be able to run python
from a command line. If you do not have python installed, you can install it
using a package manager like `brew` or `apt` if possible. If you run Windows or
do not use a package manager, you can install python directly from here
https://www.python.org/downloads/. If does not matter whether you are using
python2 or python3, and no additional packages are required to run the
configuration script.

### Development Requirements
To setup the examples or run the test cases, you are required to be in a
unix-like environment with `make` and `gcc` installed.

## Getting started
Before including MSP into your code, you must first configure MSP using the
provided `conf.py` script. This script will read arguments from the command
line and put the configured MSP library into the `target/` directory.

For more details on how you can configure MSP, run `python conf.py -h`.

### Experiment
To configure MSP for an experiment with address 0x11 and an MTU of 507, the
call to the configure script would look like this:
```
python conf.py experiment --addr=0x11 --mtu=507
```

This will generate a configured MSP library under the directory `target/`. Copy
all the files from the `target/` directory somewhere into your project.

The first step is to integrate the MSP library into your I2C driver. If you
configured MSP using the `--driver` option, you can skip this step. The way in
which you integrate the MSP library into your I2C driver is by including the
`msp_exp.h` header and making your I2C driver call the following 2 functions:
```c
/* Callback function for when receiving data from the OBC. */
int msp_recv_callback(const unsigned char *data, unsigned long len);

/* Callback function for when the OBC is requesting data. */
int msp_send_callback(unsigned char *data, unsigned long *len);
```

`msp_recv_callback` must be called directly after you have received data from
the OBC. The _data_ parameter is a pointer to a buffer where the data that was
received from the OBC is stored. The _len_ parameter specifies how many bytes
that were received from the OBC.

`msp_send_callback` must be called before you are about to send data to the
OBC. The function call will fill up a buffer pointed to by the _data_ parameter
with the bytes that you should send to the OBC. The number of bytes the were
written into the buffer are stored in the long pointed to by the parameter
_len_. The maximum number of bytes that you will ever send to or receive from
the OBC is specified by the `MSP_EXP_MAX_FRAME_SIZE` constant.

If this is done correctly, your I2C driver should now be properly integrated
with MSP. If there still are some confusions to this process, look in the file
`src/driver/msp_i2c_slave_due.c` to get an idea of how this can be done.

The second step is to interface your main code to the MSP library. This is
done by implementing 9 very simple functions (see `msp_exp_handler.h` for a
more detailed description of what each of these functions should do):
```c
void msp_expsend_start(unsigned char opcode, unsigned long *len);
void msp_expsend_data(unsigned char opcode, unsigned char *buf, unsigned long len, unsigned long offset);
void msp_expsend_complete(unsigned char opcode);
void msp_expsend_error(unsigned char opcode, int error);
void msp_exprecv_start(unsigned char opcode, unsigned long len);
void msp_exprecv_data(unsigned char opcode, const unsigned char *buf, unsigned long len, unsigned long offset);
void msp_exprecv_complete(unsigned char opcode);
void msp_exprecv_error(unsigned char opcode, int error);
void msp_exprecv_syscommand(unsigned char opcode);
```

While it might seem like a lot to implement, the behavior of these functions
should be very simple. The functions prefixed with `msp_expsend` are called in
the following manner:
1. `msp_expsend_start` is called at the start of an OBC Request transaction
   (where experiment is sending data). The _opcode_ parameter specifies the
   type of data that the OBC is requesting. The number of bytes that you
   are going to send to the OBC has to be written into the long pointed to
   by the _len_ parameter. If you have no data to send that is associated
   with the specified opcode, you can simply set `*len = 0;` in those cases.
2. `msp_expsend_data` is called when you are supposed to fill up a data
   frame with data. While it might not be needed, the _opcode_ parameter is
   still there to remind you which kind of data that the OBC is requesting. The
   _buf_ parameter is a pointer to where you insert the data. The _len_
   parameter specifies how much data you should insert. The _offset_ parameter
   specifies how many bytes of data you have inserted into previous data
   frames. **Be aware that the offset for the next invocation of
   msp_expsend_data can be the same as for the previous one if something went
   wrong during I2C transfer. So do NOT delete data as soon as you have
   inserted it into a frame.**
3. `msp_expsend_complete` is called when an OBC Request transaction is
   successfully completed.
4. `msp_expsend_error` is called when an OBC Request transaction has
   encountered an unrecoverable error and must be aborted. The type of error
   that occurred is specified by the _error_ parameter. The value of the error
   will be one of the error codes defined in `msp_exp_error.h`.

The functions prefixed with `msp_exprecv` are called in the following manner:
1. `msp_exprecv_start` is called at the start of an OBC Send transaction (where
   experiment is receiving data). The _opcode_ parameter specifies the type of
   data that the OBC is sending. The _len_ parameter specifies how much data
   the OBC is going to send. **NOTE: This is not called for opcodes that are
   classified as system commands.**
2. `msp_exprecv_data` is called each time you receive a data frame from the
   OBC. The _opcode_ is still there to remind you which type of data that the
   OBC is sending. The _buf_ parameter is a pointer to the data received from
   the data frame. The _len_ parameter specifies how much data that was
   received in this data frame. The _offset_ parameter specifies how much data
   that has been received from previous data frames. **Be aware that the offset
   for the next invocation of msp_exprecv_data can be the same as for the
   previous one if something went wrong during I2C transfer. So do NOT try to
   keep track of the data offset outside of this function.**
3. `msp_exprecv_complete` is called when an OBC Send transaction is
   successfully completed.
4. `msp_exprecv_error` is called when an OBC Send transaction has encountered
   an unrecoverable error and must be aborted. The type of error that occurred
   is specified by the _error_ parameter. The value of the error will be one of
   the error codes defined in `msp_exp_error.h`.

Opcodes that are classified as system commands are not handled by any of the
above `msp_exprecv` functions. Instead, they are handled by this single
function:
1. `msp_exprecv_syscommand` is called when a system command is sent by the OBC.
   The _opcode_ parameter specifies which system command was sent.

To help you interface your main code against MSP, all standard opcodes are
defined as constants in `msp_opcodes.h` and will be accessible after including
`msp_exp.h`. The defined names of the standard opcodes that will be set in the
_opcode_ parameter in the 9 interfacing functions are:
 - `MSP_OP_ACTIVE`
 - `MSP_OP_SLEEP`
 - `MSP_OP_POWER_OFF`
 - `MSP_OP_REQ_PAYLOAD`
 - `MSP_OP_REQ_HK`
 - `MSP_OP_REQ_PUS`
 - `MSP_OP_SEND_TIME`
 - `MSP_OP_SEND_PUS`

The third step is to make sure that your code saves the sequence flags to
non-volatile memory and restores the sequence flags at start up. The saving of
the sequence flags should be done as often as possible, but at least right
before your experiment is powered off. The saving and restoring of the sequence
flags is important due to that it is the sequence flags that keeps track of
previous transaction-ID's. If something goes wrong with saving or restoring the
sequence flags, it could result in that initial transactions contains
duplicate data or that they are discarded by the OBC.

The sequence flags are accessed through the msp state in `msp_exp_state.h`. To
access the sequence flags, just include `msp_exp.h` and the sequence flags will
be accessible through the function `msp_exp_state_get_seqflags()`. Using the
functions `write_data` and `read_data` as placeholders for reading and writing
into non-volatile memory, the procedures for saving and restoring the sequence
flags might look something like this:
```c
void restore_seqflags(void)
{
    msp_seqflags_t seqflags;

    /* Where to read from, where to store the read bytes, how many bytes to read. */
    read_data(SEQFLAGS_MEMORY_LOCATION, (unsigned char *) &seqflags, sizeof(msp_seqflags_t));

    /* You must use this function call when restoring the sequence flags,
     * otherwise there is a risk that they might get overwritten on the first
     * transaction. */
    msp_exp_state_initialize(seqflags);
}

void save_seqflags(void)
{
    msp_seqflags_t seqflags = msp_exp_state_get_seqflags();

    /* Where to write to, where to write from, how many bytes to write. */
    write_data(SEQFLAGS_MEMORY_LOCATION, (unsigned char *) &seqflags, sizeof(msp_seqflags_t));
}
```
The first time around when there is nothing to load, you do not have to do
anything. A new set of sequence flags will get initiated on the first
transaction if `msp_exp_state_initialize` has not yet been called.

That is it. Now your experiment should be up and running MSP. If there is
still some confusion to as how these functions should be implemented, please
consult the experiment example in the `examples/experiment/` directory.


### OBC
To configure MSP in OBC mode to run on an Arduino Due, the call to the
configure script would look like this:
```
python conf.py obc --driver=due
```

Unlike the experiment side of MSP, the OBC side needs a set of I2C functions
that are implemented according to `src/driver/msp_i2c_master.h`. If no driver
is specified, a template file will be included instead.

Configure script should now have generated a configured version the MSP library
under the `target/` directory. Copy all the files from the `target/` directory
somewhere into your project.

To start, include the header `msp_obc.h` into all the files where you wish to
use MSP functionalities and definitions. To communicate with an experiment
on-board the satellite, an MSP link has to be created towards it using the
function `msp_create_link`. This will create a link that you must make sure is
saved between MSP transactions as it contains information that is required for
future transactions. Look in the header `msp_obc_link.h` to see all the
functions you need to communicate over MSP.

## Directory Structure
The chart below shows the directory structure in the repository.
```
.
+-- docs
|   /* files for generating Doxygen documentation */
+-- examples
|   +-- obc
|   |   /* use case example of the OBC side of MSP */
|   +-- experiment
|       /* use case example of the experiment side of MSP */
+-- src
|   +-- common
|   |   /* files that shared between the OBC and experiment side of MSP */
|   +-- driver
|   |   /* contains I2C drivers for both experiment and OBC */
|   +-- experiment
|   |   /* files that are required for the experiment side of MSP */
|   +-- obc
|       /* files that are required for the OBC side of MSP */
+-- tests
    +-- experiment
    |   /* test cases for the experiment side of MSP */
    +-- obc
        /* test cases for the OBC side of MSP */
```
