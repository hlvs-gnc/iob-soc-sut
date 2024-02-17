/*********************************************************
 *                    Tester Firmware                    *
 *********************************************************/
#include "bsp.h"
#include "iob-axistream-in.h"
#include "iob-axistream-out.h"
#include "iob-gpio.h"
#include "iob-uart16550.h"

// System may not use ILA/PFSM for Quartus boards
#if __has_include("ILA0.h")
#include "ILA0.h" // ILA0 instance specific defines
#include "iob-ila.h"
#include "iob-pfsm.h"
#define USE_ILA_PFSM
#endif

#include "iob-dma.h"
#include "iob-eth.h"
#include "iob_soc_sut_swreg.h"
#include "iob_soc_tester_conf.h"
#include "iob_soc_tester_periphs.h"
#include "iob_soc_tester_system.h"
#include "printf.h"
#include "stdlib.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "iob_eth_rmac.h"
#define ETH_MAC_ADDR 0x01606e11020f

// Enable debug messages.
#define DEBUG 0
#define DEBUG_RX 1
#define DEBUG_TX 1
#define TEST_LOG 1
#define GPIO_OPTION 1 
#define REGFILE_OPTION 1
#define AXIS_LOOPBACK 1

#define MAX_FILE_SIZE 1048576
#define MAX_WORDS_AXIS 1
#define WORD_SIZE 4
#define SUT_FIRMWARE_SIZE 29000
#define ILA_PFSM 0

void print_ila_samples();
void send_axistream(uint32_t *byte_stream, uint8_t words_in_byte_stream);
uint8_t receive_axistream(volatile uint32_t *byte_stream);

#if (ILA_PFSM == 1)
void pfsm_program(char *);
void ila_monitor_program(char *);
#endif

void clear_cache();

unsigned char *mpeg_audio, *pcm_audio, *decoded_audio;

// Send signal by uart to receive file by ethernet
uint32_t uart_recvfile_ethernet(char *file_name) {

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

int main() {
  char pass_string[] = "Test passed!";
  char fail_string[] = "Test failed!";
  uint32_t file_size = 0;
  char c, buffer[4096], *sutStr;
  int i;

#ifndef IOB_SOC_TESTER_INIT_MEM
  char sut_firmware[SUT_FIRMWARE_SIZE];
#endif

  // Init uart0
  uart16550_init(UART0_BASE, FREQ/(16*BAUD));
  printf_init(&uart16550_putc);
  // Init SUT (connected through REGFILEIF)
  IOB_SOC_SUT_INIT_BASEADDR(SUT0_BASE);
  // Init gpio
  gpio_init(GPIO0_BASE);
  
  // init axistream
  axistream_in_init(AXISTREAMIN0_BASE);
  axistream_out_init(AXISTREAMOUT0_BASE, 4);
  axistream_in_enable();
  axistream_out_enable();

#if (ILA_PFSM == 1)
    // init integrated logic analyzer
    ila_init(ILA0_BASE);
    // Enable ILA circular buffer
    // This allows for continuous sampling while the enable signal is active
    ila_set_circular_buffer(1);
#endif

  // init dma
  dma_init(DMA0_BASE);

  // init console eth
  eth_init(ETH0_BASE, &clear_cache);

  uart16550_puts("\n[Tester]: Waiting for ethernet PHY reset to finish...\n\n");
  eth_wait_phy_rst();

#ifndef SIMULATION
  // Receive data from console via Ethernet
  file_size = uart_recvfile_ethernet("../src/eth_example.txt");
  eth_rcv_file(buffer,file_size);
  uart16550_puts("\n[Tester]: File received from console via ethernet:\n");
  for(i=0; i<file_size; i++)
    uart16550_putc(buffer[i]);
#endif

  // init SUT eth
  eth_init_mac(ETH1_BASE, ETH_RMAC_ADDR, ETH_MAC_ADDR);

  uart16550_puts("\n\n[Tester]: Hello from tester!\n\n\n");

#if (REGFILE_OPTION==1)
  // Write data to the registers of the SUT to be read by it.
  IOB_SOC_SUT_SET_REG1(64);
  IOB_SOC_SUT_SET_REG2(1024);
  uart16550_puts("[Tester]: Stored values 64 and 1024 in registers 1 and 2 of the "
            "SUT.\n\n");
#endif

#if (GPIO_OPTION==1)
  // Write a test pattern to the GPIO outputs to be read by the SUT.
  gpio_set(0x1234abcd);
  uart16550_puts("[Tester]: Placed test pattern 0x1234abcd in GPIO outputs.\n\n");
#endif

#if (ILA_PFSM == 1)
    // Program PFSM
    pfsm_program(buffer);

    // Program Monitor PFSM (internal to ILA)
    ila_monitor_program(buffer);

    // Enable all ILA triggers
    ila_enable_all_triggers();
#endif

  uart16550_puts("[Tester]: Initializing SUT via UART...\n");
  // Init and switch to uart1 (connected to the SUT)
  uart16550_init(UART1_BASE, FREQ/(16*BAUD));

  // Wait for ENQ signal from SUT
  while ((c = uart16550_getc()) != ENQ)
    if (DEBUG) {
      uart16550_base(UART0_BASE);
      uart16550_putc(c);
      uart16550_base(UART1_BASE);
    };
  
  // Send ack to sut
  uart16550_putc(ACK);

  uart16550_base(UART0_BASE);
  uart16550_puts("[Tester]: Received SUT UART enquiry and sent acknowledge.\n");
  
  uart16550_base(UART1_BASE);

#ifndef IOB_SOC_TESTER_INIT_MEM
  uart16550_base(UART0_BASE);
  uart16550_puts("[Tester]: SUT memory is not initalized. Waiting for firmware "
            "transfer request from SUT...\n");
  uart16550_base(UART1_BASE);

  // receive firmware request from SUT
  // Wait for FRX signal from SUT
  while (uart16550_getc() != FRX)
    ;
  // Receive filename
  for (i = 0; (buffer[i] = uart16550_getc()) != '\0'; i++)
    ;
  // Switch back to UART0
  uart16550_base(UART0_BASE);

  uart16550_puts("[Tester]: Received firmware transfer request with filename: ");
  uart16550_puts(buffer);
  uart16550_putc('\n');
  uart16550_puts("[Tester]: Sending transfer request to console...\n");

  // Make request to host
  file_size = uart16550_recvfile(buffer, sut_firmware);

  uart16550_puts("[Tester]: SUT firmware obtained. Transfering it to SUT via UART...\n");

  // Switch back to UART1
  uart16550_base(UART1_BASE);

  // send file size
  uart16550_putc((char)(file_size & 0x0ff));
  uart16550_putc((char)((file_size & 0x0ff00) >> 8));
  uart16550_putc((char)((file_size & 0x0ff0000) >> 16));
  uart16550_putc((char)((file_size & 0x0ff000000) >> 24));

  // Wait for ACK signal from SUT
  while (uart16550_getc() != ACK)
    ;
  if (DEBUG) {
    uart16550_base(UART0_BASE);
    uart16550_puts("[Tester] Got ack! Sending firmware to SUT...\n");
    uart16550_base(UART1_BASE);
  }

  // send file contents
  for (i = 0; i < file_size; i++)
    uart16550_putc(sut_firmware[i]);

  // Ignore firmware sent back
  uart16550_base(UART0_BASE);
  uart16550_puts("[Tester]: SUT firmware transfered. Ignoring firmware readback "
            "sent by SUT...\n");
  uart16550_base(UART1_BASE);

  // Wait for FTX signal from SUT
  while (uart16550_getc() != FTX)
    ;
  // Receive filename
  for (i = 0; (buffer[i] = uart16550_getc()) != '\0'; i++)
    ;

  // receive file size
  file_size = uart16550_getc();
  file_size |= ((uint32_t)uart16550_getc()) << 8;
  file_size |= ((uint32_t)uart16550_getc()) << 16;
  file_size |= ((uint32_t)uart16550_getc()) << 24;

  // ignore file contents received
  for (i = 0; i < file_size; i++) {
    uart16550_getc();
  }

  uart16550_base(UART0_BASE);
  uart16550_puts("[Tester]: Finished receiving firmware readback.\n");
  uart16550_base(UART1_BASE);

#endif //IOB_SOC_TESTER_INIT_MEM

  uart16550_base(UART0_BASE);

  //Delay to allow time for sut to run bootloader and enable its axistream
  for ( i = 0; i < (FREQ/BAUD)*256; i++)asm("nop");

#if (AXIS_LOOPBACK==1)
  // Allocate memory for byte stream
  uint32_t *stream_out_test = (uint32_t *) calloc(MAX_WORDS_AXIS, WORD_SIZE);

  // Fill byte stream to send
  stream_out_test[0] = 0x03020100;

  // Send byte stream via AXI stream
  send_axistream(stream_out_test, 1);

  free(stream_out_test);
#endif

#if (ILA_PFSM == 1)
    // Disable all ILA triggers
    ila_disable_all_triggers();
    
    // Print sampled ILA values
    print_ila_samples();
#endif

  // Test sending data to SUT via ethernet
  uart16550_puts("[Tester]: Sending data to SUT via ethernet:\n");
  for(i=0; i<4; i++) {
    buffer[i] = i; 
    printf("%d ", buffer[i]);
  }

  uart16550_putc('\n'); uart16550_putc('\n');
  // Send file using ethernet interface
  eth_send_file(buffer, 4);

  uart16550_puts("\n[Tester]: Reading SUT messages...\n");
  uart16550_base(UART1_BASE);

#ifdef TEST_LOG
  i = 0;
  // Read and store messages sent from SUT
  // Up until it sends the test.log file
  while ((c = uart16550_getc()) != FTX) {
    buffer[i] = c;
    if (DEBUG) {
      uart16550_base(UART0_BASE);
      uart16550_putc(c);
      uart16550_base(UART1_BASE);
    }
    i++;
  }
  buffer[i] = EOT;

  // Receive filename (test.log)
  for (i = 0; uart16550_getc() != '\0'; i++)
    ;

  // receive file size (test.log)
  file_size = uart16550_getc();
  file_size |= ((uint32_t)uart16550_getc()) << 8;
  file_size |= ((uint32_t)uart16550_getc()) << 16;
  file_size |= ((uint32_t)uart16550_getc()) << 24;

  // ignore file contents received (test.log)
  for (i = 0; i < file_size; i++)
    uart16550_getc();
#endif

  // End UART1 connection with SUT
  uart16550_finish();

  // Switch back to UART0
  uart16550_base(UART0_BASE);

  // Send messages previously stored from SUT
  uart16550_puts("[Tester]: #### Messages received from SUT: ####\n\n");
  for (i = 0; buffer[i] != EOT; i++) {
    uart16550_putc(buffer[i]);
  }
  uart16550_puts("\n[Tester]: #### End of messages received from SUT ####\n\n");

#if (REGFILE_OPTION==1)
  // Read data from the SUT's registers
  uart16550_puts("[Tester]: Reading SUT's register contents:\n");
  printf("[Tester]: Register 3: %d \n", IOB_SOC_SUT_GET_REG3());
  printf("[Tester]: Register 4: %d \n", IOB_SOC_SUT_GET_REG4());
#endif

#if (GPIO_OPTION==1)
  // Read pattern from GPIO inputs (was set by the SUT)
  printf("\n[Tester]: Pattern read from GPIO inputs: 0x%x\n\n", gpio_get());
#endif

#if (AXIS_LOOPBACK==1)
  // Allocate memory for byte stream
  volatile uint32_t *stream_in_test = (volatile uint32_t *) calloc(MAX_WORDS_AXIS, WORD_SIZE);

  // Read byte stream via AXI stream
  receive_axistream(stream_in_test);

  free((uint32_t *)stream_in_test);
#endif

#ifdef IOB_SOC_TESTER_USE_EXTMEM
  uart16550_puts("\n[Tester] Using shared external memory. Obtain SUT memory string "
            "pointer via SUT's register 5...\n");
  uart16550_puts("[Tester]: String pointer is: ");
  printf("0x%x", IOB_SOC_SUT_GET_REG5());
  uart16550_putc('\n');
  // Get address of string stored in SUT's memory
  // and invert the highest bit of MEM_ADDR_W to access the SUT's memory zone
  sutStr = (char *)(IOB_SOC_SUT_GET_REG5() ^ (1 << (IOB_SOC_TESTER_MEM_ADDR_W - 1)));

  // Print the string by accessing that address
  uart16550_puts("[Tester]: String read from SUT's memory via shared memory:\n");
  for (i = 0; sutStr[i] != '\0'; i++) {
    uart16550_putc(sutStr[i]);
  }
  uart16550_putc('\n');
#endif

#if (ILA_PFSM == 1)
    // Allocate memory for ILA output data
    const uint32_t ila_n_samples = (1<<4); //Same as buffer size
    uint32_t ila_data_size = ila_output_data_size(ila_n_samples, ILA0_DWORD_SIZE);

    // Write data to allocated memory
    uint32_t latest_sample_index = ila_number_samples();
    ila_output_data(buffer, latest_sample_index, (latest_sample_index-1)%ila_n_samples, ila_n_samples, ILA0_DWORD_SIZE);

    // Send ila data to file via UART
    uart16550_sendfile("ila_data.bin", ila_data_size-1, buffer); //Don't send last byte (\0)
#endif

  uint32_t mpeg_file_size = 0, pcm_file_size = 0, decoded_audio_size = 0;

  uart16550_puts("----------------- MPEG Decoder Testing -----------------\n");

  // Receive MPEG audio file by UART (for decoding)
  mpeg_audio = (unsigned char *) malloc(MAX_FILE_SIZE);

  mpeg_file_size = uart16550_recvfile("../src/testcase-22050.mp2", mpeg_audio);
  
  //for(int n = 0; n < 384; n++)
  //  printf("mpeg_audio[%d]: %d\n", n, mpeg_audio[n]);

  if(mpeg_file_size > MAX_FILE_SIZE){
    uart16550_puts("\n[Tester]: warning - input file size exceed limits\n");
    uart16550_puts("\n[Tester]: begin new file receive and memory allocation\n");

    free(mpeg_audio);
    mpeg_audio = (unsigned char *) malloc(mpeg_file_size);
    mpeg_file_size = uart16550_recvfile("../src/testcase-22050.mp2", mpeg_audio);
  }

  uart16550_puts("\n[Tester]: File received from console via UART\n");

  //uart16550_puts("Waiting to receive pcm file\n");
  
  // Receive pcm file by UART (for comparison with decoded data)
  //pcm_audio = mpeg_audio + mpeg_file_size;
  pcm_file_size = 13824;

  //pcm_audio = (unsigned char *) malloc(MAX_FILE_SIZE);
  //pcm_file_size = uart16550_recvfile("../src/testcase-22050.pcm", pcm_audio);

  //Memory for decoded data
  decoded_audio = (unsigned char *) malloc(MAX_FILE_SIZE);
  decoded_audio = pcm_audio + pcm_file_size;

  unsigned int bytes_sent = 0; 
  uint8_t last_percentage = 0, percentage, words_to_transfer;

  volatile uint32_t *byte_stream_in;
  uint32_t *byte_stream_out; // [MAX_WORDS_AXIS];
  uint8_t received_words;

  //Decode audio
  while(decoded_audio_size < pcm_file_size){
    // Try to send more MPEG encoded data if we still have any left
    if(!axistream_out_full() & (bytes_sent < mpeg_file_size)){
      printf("AXISTREAM_OUT_FULL: %b\n", axistream_out_full());     
      printf("AXISTREAM_FIFO_LEVEL: %b\n", axistream_out_fifo_level());

      words_to_transfer = (mpeg_file_size-bytes_sent)/WORD_SIZE >= MAX_WORDS_AXIS ? MAX_WORDS_AXIS : ((mpeg_file_size-bytes_sent)/WORD_SIZE + 1);
      
      byte_stream_out = (uint32_t *) calloc(words_to_transfer, WORD_SIZE);

      // Convert and store the values
      for(int n = 0; n < words_to_transfer; n++) {
        // Combining 4 characters into a single uint32_t value
        for(int p = 0; p < WORD_SIZE; p++)
          byte_stream_out[n] |= (mpeg_audio[n*WORD_SIZE + p + bytes_sent] << (p * 8));
      }

      send_axistream(byte_stream_out, words_to_transfer);
      
      /*
      for (;;) {
   
        if (axistream_out_fifo_level() == 0b1111) { 
          axistream_out_reset();
          axistream_out_disable();
        }
        else { 
          axistream_out_enable();
          break;
        }
      }
      */

      free(byte_stream_out);

      bytes_sent += words_to_transfer*WORD_SIZE;
		  printf("S[%d]\n\n", bytes_sent); //DEBUG
	  }
    else
      printf("AXISTREAM_OUT_FULL: %b\n", axistream_out_full());     

    //Try to receive more pcm data if there is any
    if((!axistream_in_empty())){
      printf("AXISTREAM_IN_EMPTY: %b\n", axistream_in_empty());
      byte_stream_in = (volatile uint32_t *) calloc(MAX_WORDS_AXIS, WORD_SIZE);

      received_words = receive_axistream(byte_stream_in);

      /*
      // Convert and store the values to the decoded audio array
      for(int k = 0; k < received_words; k++) {
        // Extracting each byte and converting it to a character
        for(int m = 0; m < WORD_SIZE; m++)
          decoded_audio[k*WORD_SIZE + m + decoded_audio_size] = (byte_stream_in[k] >> (m * 8)) & 0xFF;
      }

      //Check if bytes decoded are as expected
      for(i = 0; i < words_to_transfer*WORD_SIZE; i++){
        if(decoded_audio[decoded_audio_size+i]!=pcm_audio[decoded_audio_size+i]){
        printf("\nTest failed: Decoded byte at 0x%x with value 0x%x is different from PCM file value 0x%x!\n\n",decoded_audio_size+i, decoded_audio[decoded_audio_size+i],pcm_audio[decoded_audio_size+i]);
        uart16550_finish();
        return -1;
        }
      }
      */

      decoded_audio_size += received_words*WORD_SIZE;
      printf("R[%d]\n\n", decoded_audio_size); //DEBUG
      free((uint32_t *)byte_stream_in);
		}
    if(axistream_in_empty())
      printf("AXISTREAM_IN_EMPTY: %b\n", axistream_in_empty());

    //Print progress
    if((percentage = decoded_audio_size*10/pcm_file_size)>last_percentage){
      printf("%3d %%\n",percentage*10);
      last_percentage=percentage;
    }
  }

  uart16550_puts("\nTest complete! PCM file and decoded audio are equal.\n\n");

  // Send back decoded file by UART
  
  uart16550_puts("Sending decoded audio file...\n");
  char w_file[] = "testcase-22050.pcm";
  uart16550_sendfile(w_file,
                     decoded_audio_size,
                     decoded_audio);

  free(mpeg_audio);
  free(pcm_audio);
  free(decoded_audio);

  //Free memory
  axistream_out_reset();

  uart16550_puts("\n[Tester]: Verification successful!\n\n");
  uart16550_sendfile("test.log", strlen(pass_string), pass_string);

  // End UART0 connection
  uart16550_finish();
}

#if (ILA_PFSM == 1)
// Program independent PFSM peripheral of the Tester
void pfsm_program(char *bitstreamBuffer){
  // init Programmable Finite State Machine
  pfsm_init(PFSM0_BASE, 2, 1, 1);
  uint32_t file_size = 0;
  // Receive pfsm bitstream
  file_size = uart16550_recvfile("pfsm.bit", bitstreamBuffer);
  // Program PFSM
  uart16550_puts("[Tester]: Programming PFSM...\n");
  printf("[Tester]: Programmed PFSM with %d bytes.\n\n",
         pfsm_bitstream_program(bitstreamBuffer)
         );
}

// Program Monitor PFSM internal to ILA.
void ila_monitor_program(char *bitstreamBuffer){
  // init ILA Monitor (PFSM)
  pfsm_init(ila_get_monitor_base_addr(ILA0_BASE), 2, 1, 1);
  uint32_t file_size = 0;
  // Receive pfsm bitstream
  file_size = uart16550_recvfile("monitor_pfsm.bit", bitstreamBuffer);
  // Program PFSM
  uart16550_puts("[Tester]: Programming Monitor PFSM...\n");
  printf("[Tester]: Programmed Monitor PFSM with %d bytes.\n\n",
         pfsm_bitstream_program(bitstreamBuffer)
         );
}

void print_ila_samples() {
  // From the ILA0 configuration: bits 0-15 are the timestamp; bits 16-47 are fifo_value; bits 48-52 are the fifo_level; bit 53 is PFSM output
  uint32_t j, i, fifo_value;
  uint16_t initial_time = (uint16_t)ila_get_large_value(0,0);
  uint32_t latest_sample_index = ila_number_samples();
  const uint32_t ila_buffer_size = (1<<4);

  // Allocate memory for samples
  // Each buffer sample has 2 * 32 bit words
  volatile uint32_t *samples = (volatile uint32_t *)malloc((ila_buffer_size*2)*WORD_SIZE);

  // Point ila cursor to the latest sample
  ila_set_cursor(latest_sample_index,0);

  uart16550_puts("[Tester]: Storing ILA samples into memory via DMA...\n");
  dma_start_transfer((uint32_t *)samples+i, ila_buffer_size*2, 1, 1);

  clear_cache();

  uart16550_puts("[Tester]: ILA values sampled from the AXI input FIFO of SUT: \n");
  uart16550_puts("[Tester]: | Timestamp | FIFO level | AXI input value | PFSM output |\n");
  // For every sample in the buffer
  for(i=0; i<ila_buffer_size*2; i+=2){
    fifo_value = samples[i+1]<<16 | samples[i]>>16;
    printf("[Tester]: | %06d    | 0x%02x       | 0x%08x      | %d           |\n",(uint16_t)(samples[i]-initial_time), samples[i+1]>>16 & 0x1f, fifo_value, samples[i+1]>>21 & 0x1);
  }
  uart16550_putc('\n');

  free((uint32_t *)samples);
}
#endif

void send_axistream(uint32_t *byte_stream, uint8_t words_in_byte_stream) {
  uint8_t i;

  uart16550_puts("[Tester]: Sending AXI stream bytes\n");

#if (DEBUG_TX==1)
  // Print byte stream to send
  // Convert and store the values to the decoded audio array
  for(int k = 0; k < words_in_byte_stream; k++) {
    // Extracting each byte and converting it to a character
    for(int m = 0; m < WORD_SIZE; m++)
      printf("0x%02x ",  (byte_stream[k] >> (m * 8)) & 0xFF);
  }
  uart16550_puts("\n");
#endif

  // Send bytes to AXI stream output via DMA, except the last word.
  dma_start_transfer(byte_stream, words_in_byte_stream-1, 0, 0);
  // Send the last word with via SWregs with the TLAST signal.
  axistream_out_push(byte_stream[words_in_byte_stream-1], 1, 1);
}

uint8_t receive_axistream(volatile uint32_t *byte_stream) {
  uint8_t i = 0, rstrb;

  uint8_t n_received_words = axistream_in_fifo_level();

  if(!axistream_in_empty()){

    // Transfer bytes from AXI stream input via DMA
    uart16550_puts("[Tester]: Storing AXI words via DMA\n");
    dma_start_transfer((uint32_t *)byte_stream+i, n_received_words, 1, 0);

    clear_cache();

    uart16550_puts("[Tester]: Received AXI stream bytes\n");

#if (DEBUG_RX==1)
    // Print byte stream received
    for (i = 0; i < n_received_words*WORD_SIZE; i++)
      printf("0x%02x ", ((volatile uint8_t *)byte_stream)[i]);

    uart16550_puts("\n");
#endif
  }

  return n_received_words;

}

void clear_cache(){
  // Delay to ensure all data is written to memory
  for ( unsigned int i = 0; i < 10; i++)asm volatile("nop");
  // Flush VexRiscv CPU internal cache
  asm volatile(".word 0x500F" ::: "memory");
}
