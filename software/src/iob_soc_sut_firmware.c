#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "bsp.h"
#include "iob_soc_sut_system.h"
#include "iob_soc_sut_periphs.h"
#include "iob_soc_sut_conf.h"
#include "iob-uart16550.h"
#include "iob-gpio.h"
#include "iob-eth.h"
#include "printf.h"
#include "iob_regfileif_inverted_swreg.h"
#include "iob-axistream-in.h"
#include "iob-axistream-out.h"

#include "mad.h"

#if __has_include("iob_soc_tester_conf.h")
#define USE_TESTER
#endif

// Enable debug messages.
#define DEBUG 1
#define TEST_LOG 1
#define GPIO_OPTION 1 
#define REGFILE_OPTION 1
#define AXIS_LOOPBACK 1

#define INPUTBUFFERSIZE 4096
#define NBR_BYTES 1024

#define MAX_WORDS_AXIS 1
#define WORD_SIZE 4
#define MAX_AXIS_BYTES MAX_WORDS_AXIS*WORD_SIZE

void axistream_loopback();

void clear_cache(){
  // Delay to ensure all data is written to memory
  for ( unsigned int i = 0; i < 10; i++)asm volatile("nop");
  // Flush VexRiscv CPU internal cache
  asm volatile(".word 0x500F" ::: "memory");
}

char pass_string[] = "Test passed!";
char fail_string[] = "Test failed!";

/*
 * This is a private message structure. A generic pointer to this structure
 * is passed to each of the callback functions. Put here any data you need
 * to access from within the callbacks.
 */

struct buffer {
  unsigned char const *start;
  unsigned long length;
};

unsigned char input_buffer[INPUTBUFFERSIZE];
struct mad_decoder decoder;

/*
 * This is the input callback. The purpose of this callback is to (re)fill
 * the stream buffer which is to be decoded. 
 * It reads data from the AXI4 Stream In peripheral.
 */

static
enum mad_flow input(void *data,
		    struct mad_stream *stream)
{
  volatile uint32_t *byte_stream_in;
  uint8_t i, j, rstrb, r_words;

  unsigned long bufferUnprocessedData = 0;

  // Check if some data was consumed
  if(stream->next_frame){

	  bufferUnprocessedData = stream->bufend - stream->next_frame;

  for(i = 0; i<bufferUnprocessedData; i++)
    input_buffer[i] = stream->next_frame[i];
  }

  while(axistream_in_empty());

  //Fill buffer with new data
  if(!axistream_in_empty() && (bufferUnprocessedData+MAX_AXIS_BYTES-1 < INPUTBUFFERSIZE)){
    
    byte_stream_in = (volatile uint32_t *) calloc(MAX_WORDS_AXIS, WORD_SIZE);
    
    // Receive bytes while stream does not end (by TLAST signal), or up to 16 32-bit words
    for(r_words = 0, i = 0; i < 1 && r_words < MAX_WORDS_AXIS; r_words++){
      byte_stream_in[r_words] = axistream_in_pop(&rstrb, &i);
      // Extracting each byte and converting it to a an unsigned integer
      for(j = 0; j < WORD_SIZE; j++)
        input_buffer[r_words*WORD_SIZE + j + bufferUnprocessedData] = (byte_stream_in[r_words] >> (j * 8)) & 0xFF;
    }

    bufferUnprocessedData += r_words*j;

  }

  free((uint32_t *) byte_stream_in);

  //Set new buffer pointers
  mad_stream_buffer(stream, input_buffer, bufferUnprocessedData);

  return MAD_FLOW_CONTINUE;
}

/*
 * The following utility routine performs simple rounding, clipping, and
 * scaling of MAD's high-resolution samples down to 16 bits. It does not
 * perform any dithering or noise shaping, which would be recommended to
 * obtain any exceptional audio quality. It is therefore not recommended to
 * use this routine if high-quality output is desired.
 */

static inline
signed int scale(mad_fixed_t sample)
{
  /* round */
  sample += (1L << (MAD_F_FRACBITS - 16));

  /* clip */
  if (sample >= MAD_F_ONE)
    sample = MAD_F_ONE - 1;
  else if (sample < -MAD_F_ONE)
    sample = -MAD_F_ONE;

  /* quantize */
  return sample >> (MAD_F_FRACBITS + 1 - 16);
}

/*
 * This is the output callback function. It is called after each frame of
 * MPEG audio data has been completely decoded. The purpose of this callback
 * is to output (or play) the decoded PCM audio.
 * It outputs frame data to the AXI4 Stream Out peripheral.
 */

static
enum mad_flow output(void *data,
		     struct mad_header const *header,
		     struct mad_pcm *pcm)
{
  unsigned int nchannels, nsamples;
  mad_fixed_t const *left_ch, *right_ch;
  
  int i;
  uint8_t sample_cnt = 0;

  /* pcm->samplerate contains the sampling frequency */

  nchannels = pcm->channels;
  nsamples  = pcm->length;
  left_ch   = pcm->samples[0];
  right_ch  = pcm->samples[1];

  uint32_t *stream_out_test = (uint32_t *) calloc(MAX_WORDS_AXIS, WORD_SIZE);
  stream_out_test[0] = 0x03020100;

  for(i = 0; i < 3; i++)
    axistream_out_push(stream_out_test[i],1,0);
  axistream_out_push(stream_out_test[i],1,1); // Send the last word with the TLAST signal

  free(stream_out_test);

  while (nsamples--) {
    signed int sample;
	  int8_t sample_bytes[MAX_WORDS_AXIS*WORD_SIZE];

    /* output sample(s) in 16-bit signed little-endian PCM */

    sample = scale(*left_ch++);
	  sample_bytes[sample_cnt] = sample & 0xff;
	  sample_bytes[sample_cnt + 1] = sample>>8 & 0xff;

    sample_cnt += 2;

    //while(axistream_out_full());
    //axistream_out_push((uint32_t) sample_bytes, 1, nsamples<1 && nchannels<2);

    if (nchannels == 2) {
      sample = scale(*right_ch++);
		  sample_bytes[sample_cnt] = sample & 0xff;
		  sample_bytes[sample_cnt + 1] = sample>>8 & 0xff;
      sample_cnt += 2;

      //while(axistream_out_full());
      //axistream_out_push((uint32_t) sample_bytes, 1, nsamples<1);
    }

    while(axistream_out_full());

    if(sample_cnt == MAX_WORDS_AXIS*WORD_SIZE){
      // Send bytes to AXI stream output
      for(i = 0; i < sample_cnt-1; i++)
        axistream_out_push(sample_bytes[i],1,0);
      axistream_out_push(sample_bytes[i],1,1); // Send the last word with the TLAST signal
      
      sample_cnt = 0;

    }
  }

  return MAD_FLOW_CONTINUE;
}

/*
 * This is the error callback function. It is called whenever a decoding
 * error occurs. The error is indicated by stream->error; the list of
 * possible MAD_ERROR_* errors can be found in the mad.h (or stream.h)
 * header file.
 */

static
enum mad_flow error(void *data,
		    struct mad_stream *stream,
		    struct mad_frame *frame)
{
  struct buffer *buffer = data;
  int error = stream->error;
  int byte_offset = stream->this_frame - buffer->start;
  
  /* return MAD_FLOW_BREAK here to stop decoding (and propagate an error) */

  return MAD_FLOW_BREAK;
}

// Send signal by uart to receive file by ethernet
uint32_t uart16550_recvfile_ethernet(char *file_name) {

  uart16550_puts(UART_PROGNAME);
  uart16550_puts (": requesting to receive file by ethernet\n");

  //send file receive by ethernet request
  uart16550_putc (0x13);

  //send file name (including end of string)
  uart16550_puts(file_name); uart16550_putc(0);

  // receive file size
  uint32_t file_size = uart16550_getc();
  file_size |= ((uint32_t)uart16550_getc()) << 8;
  file_size |= ((uint32_t)uart16550_getc()) << 16;
  file_size |= ((uint32_t)uart16550_getc()) << 24;

  // send ACK before receiving file
  uart16550_putc(ACK);

  return file_size;
}

int main()
{
  int i;
  char buffer[64];
  char file_buffer[256];
  int ethernet_connected = 0;
  int result;

  //init uart
  uart16550_init(UART0_BASE, FREQ/(16*BAUD));
  printf_init(&uart16550_putc);
  //init regfileif
  IOB_REGFILEIF_INVERTED_INIT_BASEADDR(REGFILEIF0_BASE);
  //init gpio
  gpio_init(GPIO0_BASE);
  //init axistream
  axistream_in_init(AXISTREAMIN0_BASE);   
  axistream_out_init(AXISTREAMOUT0_BASE, 4);
  axistream_in_enable();
  axistream_out_enable();
  // init eth
  eth_init(ETH0_BASE, &clear_cache);

  // Wait for PHY reset to finish
  eth_wait_phy_rst();
  
#ifdef USE_TESTER
  // Receive a special string message from tester to tell if its running linux
  char tester_run_type[] = "TESTER_RUN_";
  for ( i = 0; i < 11; ) {
    if (uart16550_getc() == tester_run_type[i])
      i++;
    else
      i = 0;
  }
  char tester_run_type2[] = "LINUX";
  int tester_run_linux = 1;
  for ( i = 0; i < 5; ) {
    if (uart16550_getc() == tester_run_type2[i]) {
      i++;
    } else {
      tester_run_linux = 0;
      break;
    }
  }

  if (!tester_run_linux) { //Ethernet does not work on Linux yet
    uart16550_puts("[SUT]: Tester running on bare metal\n");
    // Receive data from Tester via Ethernet
    ethernet_connected = 1;
    eth_rcv_file(buffer, 64);

    //Delay to allow time for tester to print debug messages
    for ( i = 0; i < (FREQ/BAUD)*128; i++)asm("nop");
  } else {
    uart16550_puts("[SUT]: Tester running on Linux\n");
  }
#else //USE_TESTER
#ifndef SIMULATION
  // Receive data from console via Ethernet
  uint32_t file_size;
  file_size = uart16550_recvfile_ethernet("../src/eth_example.txt");
  eth_rcv_file(file_buffer,file_size);
  uart16550_puts("\n[MPEG-Decoder]: File received from console via ethernet:\n");
  for(i=0; i<file_size; i++)
    uart16550_putc(file_buffer[i]);
#endif //SIMULATION
#endif //USE_TESTER

  //Write to UART0 connected to the Tester.
  uart16550_puts("\n\n[MPEG-Decoder]: This message was sent from SUT!\n\n");

  if(ethernet_connected){
    uart16550_puts("[MPEG-Decoder]: Data received via ethernet:\n");
    for(i=0; i<4; i++)
      printf("%d ", buffer[i]);
    uart16550_putc('\n'); uart16550_putc('\n');
  }
  
#if (REGFILE_OPTION==1)
  //Print contents of REGFILEIF registers 1 and 2
  uart16550_puts("[MPEG-Decoder]: Reading REGFILEIF contents:\n");
  printf("[MPEG-Decoder]: Register 1: %d \n", IOB_REGFILEIF_INVERTED_GET_REG1());
  printf("[MPEG-Decoder]: Register 2: %d \n\n", IOB_REGFILEIF_INVERTED_GET_REG2());

  //Write data to the registers of REGFILEIF to be read by the Tester.
  IOB_REGFILEIF_INVERTED_SET_REG3(128);
  IOB_REGFILEIF_INVERTED_SET_REG4(2048);
  uart16550_puts("[MPEG-Decoder]: Stored values 128 and 2048 in REGFILEIF registers 3 and 4.\n\n");
#endif

#if (GPIO_OPTION==1)
  //Print contents of GPIO inputs 
  printf("[MPEG-Decoder]: Pattern read from GPIO inputs: 0x%x\n\n",gpio_get());

  //Write the same pattern to GPIO outputs
  gpio_set(0xabcd1234);
  uart16550_puts("[MPEG-Decoder]: Placed test pattern 0xabcd1234 in GPIO outputs.\n\n");
#endif

#if (AXIS_LOOPBACK==1)
  // Read AXI stream input and relay data to AXI stream output
  axistream_loopback();
#endif

#ifdef IOB_SOC_SUT_USE_EXTMEM
  char sutMemoryMessage[]="Shared memory content is shared\n";

  uart16550_puts("\n[MPEG-Decoder]: Using external memory. Stored a string in memory at location: ");
  printf("0x%x\n", (int)sutMemoryMessage);
  
  //Give address of stored message to Tester using regfileif register 4
  IOB_REGFILEIF_INVERTED_SET_REG5((int)sutMemoryMessage);
  uart16550_puts("[MPEG-Decoder]: Stored string memory location in REGFILEIF register 5.\n");
#endif

#ifdef IOB_SOC_SUT_USE_EXTMEM
  //if(memory_access_failed)
  //    uart16550_sendfile("test.log", strlen(fail_string), fail_string);
  //    uart16550_finish();
#endif

  uart16550_puts("------------- MPEG Decoder Initialization -------------\n");

  /* configure input, output, and error functions */
  mad_decoder_init(&decoder, 0 /* private message struct */,
			             input, 0 /* header */, 0 /* filter */, output,
			             error, 0 /* message */);

#ifdef TEST_LOG
  uart16550_sendfile("test.log", strlen(pass_string), pass_string);
#endif

  uart16550_finish();

  /*
  * The SUT waits for the start signal to be reset, then
  * when start signal is active, get the input buffer size and address with
  * encoded MPEG data to process. The output buffer address defines where to 
  * place the decoded data. The output buffer size and the finished signal are
  * updated at the end of the decoding process.
  */

	// Start decoding data
	result = mad_decoder_run(&decoder, MAD_DECODER_MODE_SYNC);

  mad_decoder_finish(&decoder);

}

// Read AXI stream input, print, and relay data to AXI stream output
void axistream_loopback(){
  uint32_t byte_stream[MAX_WORDS_AXIS];
  uint8_t i, rstrb, received_words;
  
  //Check if we are receiving an AXI stream
  if(!axistream_in_empty()){
    
    // Receive bytes while stream does not end (by TLAST signal), or up to 16 32-bit words
    for(received_words = 0, i = 0; i < 1 && received_words < MAX_WORDS_AXIS; received_words++)
      byte_stream[received_words] = axistream_in_pop(&rstrb, &i);

    // Print received bytes
#if (DEBUG==1)
    uart16550_puts("[MPEG-Decoder]: Received AXI stream bytes: ");

    for(i = 0; i < received_words*WORD_SIZE; i++)
      printf("0x%02x ", ((uint8_t *)byte_stream)[i]);
#endif

    // Send bytes to AXI stream output
    for(i = 0; i < received_words-1; i++)
      axistream_out_push(byte_stream[i],1,0);
    axistream_out_push(byte_stream[i],1,1); // Send the last word with the TLAST signal
    
#if (DEBUG==1)
    uart16550_puts("\n[MPEG-Decoder]: Sent AXI stream bytes back via output interface.\n\n");
  } else {
    // Input AXI stream queue is empty
    uart16550_puts("[MPEG-Decoder]: AXI stream input is empty. Skipping AXI stream tranfer.\n\n");
#endif
  }
}
