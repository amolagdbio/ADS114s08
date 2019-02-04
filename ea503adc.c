/**
 * @file   armdriver.c
 * @author Amol Palshetkar
 * @date   03 November 2018
 * @brief  A kernel module for controlling amr up & down motion using opto and steps.
 * It is threaded in order that it can control the motors.
 * The sysfs entry appears at /sys/armdriver/arm

 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>       	// Required for the GPIO functions
#include <linux/kobject.h>    	// Using kobjects for the sysfs bindings
#include <linux/hrtimer.h>		// Used by high resolution timer
//#include <linux/ktime.h>		// Used by high resolution timer
#include <linux/interrupt.h>  	// Required for the IRQ code
#include <linux/delay.h>      	// Using this header for the msleep() function

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Amol Palshetkar");
MODULE_DESCRIPTION("Turbolyte ADC driver for reading electrodes data");
MODULE_VERSION("0.1");

// Register Map
#define ADS_ID			0x00
#define ADS_STATUS		0x01
#define ADS_INPMUX		0x02
#define ADS_PGA			0x03
#define ADS_DATARATE	0x04
#define ADS_REF			0x05
#define ADS_IDACMAG		0x06
#define ADS_IDACMUX		0x07
#define ADS_VBIAS		0x08
#define ADS_SYS			0x09
#define ADS_RESERVED0	0x0A
#define ADS_OFCAL0		0x0B
#define ADS_OFCAL1		0x0C
#define ADS_RESERVED1	0x0D
#define ADS_FSCAL0		0x0E
#define ADS_FSCAL1		0x0F
#define ADS_GPIODAT		0x10
#define ADS_GPIOCON		0x11

// Commands
#define CMD_NOP			0x00
#define CMD_WAKEUP		0x02
#define CMD_POWERDN		0x04
#define CMD_RESET		0x06
#define CMD_START		0x08
#define CMD_STOP		0x0A
#define CMD_SYOCAL		0x16
#define CMD_SYGCAL		0x17
#define CMD_SFOCAL		0x19
#define CMD_RDATA		0x12
#define CMD_RREG		0x20
#define CMD_WREG		0x40

enum acn {
	CH0 = 0x0c,
	CH1 = 0x1c,
	CH2 = 0x2c,
	CH3 = 0x3c,
	CH4 = 0x4c,
	CH5 = 0x5c,
	CH6 = 0x6c,
	CH7 = 0x7c,
	CH8 = 0x8c,
	CH9 = 0x9c,
	CH10 = 0xac,
	CH11 = 0xbc,
	};
	
static bool isRising = 1;                   ///< Rising edge is the default IRQ property

static uint8_t rec_buf[20];
static int    irqNumberMISO;                    //< Used to share the IRQ number within this file
//static uint16_t adc_buf[16][50];
static unsigned long xIntCount=0;
static enum acn adcChNo = CH5;
//static ktime_t ktime;
//static struct hrtimer hr_timer;


// SPI signals
static unsigned int SPI0_CS0 = 84;
static unsigned int SPI0_CLK = 87;
static unsigned int SPI0_MOSI = 86;
static unsigned int SPI0_MISO = 85;

static unsigned int adcChannel = 0;
//static unsigned long freqSPI = 740000L;		// Frequency of spi
static int16_t adcVoltage = 0;

/// Function prototype for the custom IRQ handler function -- see below for the implementation
//enum hrtimer_restart my_hrtimer_callback( struct hrtimer *timer);
static irq_handler_t  miso_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs);

static void adc_initialize(void);
void byteOut(uint8_t byte);
uint8_t getByte(void);
static void setADCRegister(uint8_t regNo,uint8_t value);
static uint8_t getRegsiter(uint8_t regNo);
void readADCID(void);
void readAllADCRegisters(void);
int16_t readADCData(uint8_t chNo);

/** @brief A callback function to display the motor steps */
/**static ssize_t steps_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf){
  return sprintf(buf, "%ld\n", noOfSteps);
  }*/

/** @brief A callback function to store the motor steps value */
/**static ssize_t steps_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count){
  unsigned long period;                     // Using a variable to validate the data sent
  sscanf(buf, "%ldu", &period);             // Read in the period as an unsigned long
  if ((period>0)&&(period<=10000)){        // Must be 1 or greater, and less than 10000
  noOfSteps = period;                 // Within range, assign to noOfSteps variable
  }
  return period;
  }*/

/** @brief A callback function to display the SPI channel number
 *  @param kobj represents a kernel object device that appears in the sysfs filesystem
 *  @param attr the pointer to the kobj_attribute struct
 *  @param buf the buffer to which to write the number of presses
 *  @return return the number of characters of the mode string successfully displayed
 */
static ssize_t adcchannel_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf){
	printk(KERN_INFO "ADC Driver: adcchannel_show: %02x\n",getRegsiter(ADS_ID));
	return sprintf(buf,"%d\n",adcChannel);
	/*switch(adcChannel){
		case 0:		return sprintf(buf, "0\n");       // Display the state -- simplistic approach
		case 1:    	return sprintf(buf, "1\n");
		case 2:		return sprintf(buf, "2\n");
		case 3:		return sprintf(buf, "3\n");
		case 4:		return sprintf(buf, "4\n");       // Display the state -- simplistic approach
		case 5:    	return sprintf(buf, "5\n");
		case 6:		return sprintf(buf, "6\n");
		case 7:		return sprintf(buf, "7\n");
		case 8:		return sprintf(buf, "8\n");       // Display the state -- simplistic approach
		case 9:    	return sprintf(buf, "9\n");
		case 10:		return sprintf(buf, "10\n");
		case 11:		return sprintf(buf, "11n");
		default:    return sprintf(buf, "Invalid ADC channel number.\n"); // Cannot get here
	}*/
}

/** @brief A callback function to store the arm state using the enum above */
static ssize_t adcchannel_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count){
	// the count-1 is important as otherwise the \n is used in the comparison

	if (strncmp(buf,"0",count)==0) {// strncmp() compare with fixed number chars
		adcChNo = CH0;
		adcChannel = 0;
	}
	else if (strncmp(buf,"1",count)==0) {
		adcChNo = CH1;
		adcChannel = 1;
	}
	else if (strncmp(buf,"2",count)==0) {
		adcChNo = CH2;
		adcChannel = 2;
	}
	else if (strncmp(buf,"3",count)==0) {
		adcChNo = CH3;
		adcChannel = 3;
	}
	else if (strncmp(buf,"4",count)==0) {// strncmp() compare with fixed number chars
		adcChNo = CH4;
		adcChannel = 4; 
	}
	else if (strncmp(buf,"5",count)==0) {
		adcChNo = CH5;
		adcChannel = 5;
	}
	else if (strncmp(buf,"6",count)==0) {
		adcChNo = CH6;
		adcChannel = 6;
	}
	else if (strncmp(buf,"7",count-1)==0) {
		adcChNo = CH7;
		adcChannel = 7;
	}
	else if (strncmp(buf,"8",count-1)==0) {// strncmp() compare with fixed number chars
		adcChNo = CH8;
		adcChannel = 8; 
	}
	else if (strncmp(buf,"9",count-1)==0) {
		adcChNo = CH9;
		adcChannel = 9;
	}
	else if (strncmp(buf,"10",count-1)==0) {
		adcChNo = CH10;
		adcChannel = 10;
	}
	else if (strncmp(buf,"11",count-1)==0) {
		adcChNo = CH11;
		adcChannel = 11;
	}
//	printk(KERN_INFO "ADC Driver: adcchannel_store: %s %02x %d %d\n",buf,adcChNo,adcChannel,count);
	//setADCRegister(ADS_IDACMUX,adcChNo);
	//getRegsiter(ADS_IDACMUX);
	return count;
}


/** @brief A callback function to display the motor steps */
/*static ssize_t freq_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf){
	return sprintf(buf, "%ld\n", freqSPI);
}*/

/** @brief A callback function to store the motor steps value */
/*static ssize_t freq_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count){
	unsigned long period;                     // Using a variable to validate the data sent
	sscanf(buf, "%ldu", &period);             // Read in the period as an unsigned long
	//	if ((period>100000)&&(period<=1000000)){        // Must be 100000 or greater, and less than 740000
	freqSPI = period;                 // Within range, assign to freqMotor variable
	ktime = ktime_set( 0, freqSPI );
	//	}
	return period;
}*/

/** @brief A callback function to display the selected channel voltage */
static ssize_t adcVoltage_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf){
	//readAllADCRegisters();
	adcVoltage = readADCData(adcChNo);
	return sprintf(buf, "%d\n", adcVoltage);
}

/** Use these helper macros to define the name and access levels of the kobj_attributes
 *  The kobj_attribute has an attribute attr (name and mode), show and store function pointers
 *  The period variable is associated with the blinkPeriod variable and it is to be exposed
 *  with mode 0666 using the period_show and period_store functions above
 */
static struct kobj_attribute adcChannel_attr = __ATTR(adcChannel, 0600, adcchannel_show, adcchannel_store);
//static struct kobj_attribute freq_attr = __ATTR(freqSPI, 0600, freq_show, freq_store);
static struct kobj_attribute adcVoltage_attr = __ATTR(adcVoltage, 0600, adcVoltage_show, NULL);


/** The adc_attrs[] is an array of attributes that is used to create the attribute group below.
 *  The attr property of the kobj_attribute is used to extract the attribute struct
 */
static struct attribute *adc_attrs[] = {
	&adcChannel_attr.attr,						// Motor current state
//	&freq_attr.attr,						// Freq of motor
	&adcVoltage_attr.attr,						// Adc voltage of selected channel
	NULL,
};

/** The attribute group uses the attribute array and a name, which is exposed on sysfs -- in this
 *  case it is adc, which is automatically defined in the arm_init() function below
 *  using the custom kernel parameter that can be passed when the module is loaded.
 */
static struct attribute_group attr_group = {
	.attrs = adc_attrs,                      // The attributes array defined just above
};

static struct kobject *adc_kobj;            /// The pointer to the kobject


/** @brief The LKM initialization function
 *  The static keyword restricts the visibility of the function to within this C file. The __init
 *  macro means that for a built-in driver (not a LKM) the function is only used at initialization
 *  time and that it can be discarded and its memory freed up after that point. In this example this
 *  function sets up the GPIOs and the IRQ
 *  @return returns 0 if successful
 */
static int __init adc_init(void){
	int result = 0;
	unsigned long IRQflags = IRQF_TRIGGER_RISING;      // The default is a rising-edge interrupt

	printk(KERN_INFO "ADC Driver: Initializing the ADC Driver LKM\n");

	adc_kobj = kobject_create_and_add("adcdriver", kernel_kobj->parent); // kernel_kobj points to /sys/kernel
	if(!adc_kobj){
		printk(KERN_ALERT "ADC Driver: failed to create kobject\n");
		return -ENOMEM;
	}
	// add the attributes to /sys/arm/ -- for example, /sys/armdriver/arm
	result = sysfs_create_group(adc_kobj, &attr_group);
	if(result) {
		printk(KERN_ALERT "ADC Driver: failed to create sysfs group\n");
		kobject_put(adc_kobj);                // clean up -- remove the kobject sysfs entry
		return result;
	}

	gpio_request(SPI0_CS0, "sysfs");        // SPI0_CS0 is 84 by default, request it
	gpio_direction_output(SPI0_CS0, true);  // Set the gpio to be in output mode and turn on
	gpio_set_value(SPI0_CS0, true);         // Not required as set by line above (here for reference)
	gpio_export(SPI0_CS0, false);			// causes motorEnableY to appear in /sys/class/gpio
	// the second argument prevents the direction from being changed

	gpio_request(SPI0_CLK, "sysfs");          // SPI0_CLK is 87 by default, request it
	gpio_direction_output(SPI0_CLK, true);   // Set the gpio to be in output mode and turn on
	gpio_set_value(SPI0_CLK, false);          // Not required as set by line above (here for reference)
	gpio_export(SPI0_CLK, false);  // causes motorStepsY to appear in /sys/class/gpio
	// the second argument prevents the direction from being changed

	gpio_request(SPI0_MOSI, "sysfs");          // SPI0_MOSI is 86 by default, request it
	gpio_direction_output(SPI0_MOSI, true);   // Set the gpio to be in output mode and turn on
	gpio_set_value(SPI0_MOSI, false);          // Not required as set by line above (here for reference)
	gpio_export(SPI0_MOSI, false);  // causes motorStepsY to appear in /sys/class/gpio
	// the second argument prevents the direction from being changed

	gpio_request(SPI0_MISO, "sysfs");          // SPI0_MISO is 85 by default, request it
	gpio_direction_input(SPI0_MISO);   // Set the gpio to be in input mode and turn on
	gpio_export(SPI0_MISO, false);  // causes motorStepsY to appear in /sys/class/gpio
	// the second argument prevents the direction from being changed

	irqNumberMISO = gpio_to_irq(SPI0_MISO);
	if(!isRising){                           // If the kernel parameter isRising=0 is supplied
		IRQflags = IRQF_TRIGGER_FALLING;      // Set the interrupt to be on the falling edge
	}
	result = request_irq(irqNumberMISO,             // The interrupt number requested
			(irq_handler_t) miso_irq_handler, // The pointer to the handler function below
			IRQflags,              // Use the custom kernel param to set interrupt type
			"MISO_interrupt_handler",  // Used in /proc/interrupts to identify the owner
			NULL);                 // The *dev_id for shared interrupt lines, NULL is okay

	printk(KERN_INFO "HR Timer module installing\n");
//	ktime = ktime_set( 0, freqSPI );
//	hrtimer_init( &hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
//	hr_timer.function = &my_hrtimer_callback;
	
	readADCID();
	adcChNo = CH0;
	adcChannel = 0;
	adc_initialize();
	readADCData(CH0);
	
	return result;
}

/** @brief The LKM cleanup function
 *  Similar to the initialization function, it is static. The __exit macro notifies that if this
 *  code is used for a built-in driver (not a LKM) that this function is not required.
 */
static void __exit adc_exit(void){
	//int ret;
	kobject_put(adc_kobj);                   // clean up -- remove the kobject sysfs entry

	free_irq(irqNumberMISO, NULL);               // Free the IRQ number, no *dev_id required in this case

	gpio_set_value(SPI0_CS0, true);              // Turn the step off, indicates device was unloaded
	gpio_unexport(SPI0_CS0);                  // Unexport the GPIO
	gpio_free(SPI0_CS0);                      // Free the GPIO

	gpio_set_value(SPI0_CLK, false);              // Turn the step off, indicates device was unloaded
	gpio_unexport(SPI0_CLK);                  // Unexport the GPIO
	gpio_free(SPI0_CLK);                      // Free the GPIO

	gpio_set_value(SPI0_MOSI, false);              // Turn the step off, indicates device was unloaded
	gpio_unexport(SPI0_MOSI);                  // Unexport the GPIO
	gpio_free(SPI0_MOSI);                      // Free the GPIO

	gpio_unexport(SPI0_MISO);                  // Unexport the GPIO
	gpio_free(SPI0_MISO);                      // Free the GPIO

	//ret = hrtimer_cancel( &hr_timer );
	//if (ret) printk(KERN_INFO "The timer was still in use...\n");

	printk(KERN_INFO "HR Timer module uninstalling\n");

	printk(KERN_INFO "ADC Driver: Goodbye from the ADC Driver LKM!\n");
}
/** @brief The GPIO IRQ Handler function
 *  This function is a custom interrupt handler that is attached to the GPIO above. The same interrupt
 *  handler cannot be invoked concurrently as the interrupt line is masked out until the function is complete.
 *  This function is static as it should not be invoked directly from outside of this file.
 *  @param irq    the IRQ number that is associated with the GPIO -- useful for logging.
 *  @param dev_id the *dev_id that is provided -- can be used to identify which device caused the interrupt
 *  Not used in this example as NULL is passed.
 *  @param regs   h/w specific register values -- only really ever used for debugging.
 *  return returns IRQ_HANDLED if successful -- should return IRQ_NONE otherwise.
 */
static irq_handler_t miso_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs){
//	optoXState = gpio_get_value(SPI0_MISO);
//	mdelay(1000);
//	if (optoXState == BLOCK){
		xIntCount++;
//	}
	/*printk(KERN_INFO "ARM Driver: The optoX state is currently: %d\n", optoXState);*/
	return (irq_handler_t) IRQ_HANDLED;  // Announce that the IRQ has been handled correctly
}

/*enum hrtimer_restart my_hrtimer_callback( struct hrtimer *timer ){

	//return HRTIMER_NORESTART;
	return HRTIMER_RESTART;

}*/
static void setADCRegister(uint8_t regNo,uint8_t value){
	rec_buf[regNo] = value;
	gpio_set_value(SPI0_CS0,0);
	byteOut(CMD_WREG|regNo);
	byteOut(1);
	byteOut(value);
	gpio_set_value(SPI0_CS0,1);
}


static uint8_t getRegsiter(uint8_t regNo){
	uint8_t recvbyte;
	gpio_set_value(SPI0_CS0,0);
	byteOut(CMD_RREG|regNo);
	byteOut(0x01);
	recvbyte = getByte();
	gpio_set_value(SPI0_CS0,1);
	printk(KERN_INFO "ADC Driver: getRegsiter(%02x): %02x\n",regNo,recvbyte);
	return recvbyte;
}

/**
 * @brief Arm homing function
 * The function brings the arm to Home position above bottom seal adapter.
 */
static void adc_initialize(void){
	uint8_t cnt=0;
	xIntCount = 0;
	
	readAllADCRegisters();
	
	rec_buf[ADS_INPMUX] 	= adcChNo;	// ADS_INPMUX	Select channel 0
	rec_buf[ADS_PGA] 		= 0x00;		// ADS_PGA	Enable single ended measurement & PGA gain = 1
	rec_buf[ADS_DATARATE] 	= 0x34;//0x14;		// ADS_DATARATE	Continous conversion, 20 Samples per second
	rec_buf[ADS_REF] 		= 0x32;//30		// ADS_REF
	rec_buf[ADS_IDACMAG] 	= 0x00;		// ADS_IDACMAG
	rec_buf[ADS_IDACMUX] 	= 0xFF;		// ADS_IDACMUX
	rec_buf[ADS_VBIAS] 		= 0x00;		// ADS_VBIAS
	rec_buf[ADS_SYS] 		= 0x10;		// ADS_SYS
	rec_buf[ADS_RESERVED0] 	= 0x00;		// ADS_RESERVED0
	rec_buf[ADS_OFCAL0] 	= 0x00;		// ADS_OFCAL0
	rec_buf[ADS_OFCAL1] 	= 0x00;		// ADS_OFCAL1
	rec_buf[ADS_RESERVED1] 	= 0x00;		// ADS_RESERVED1
	rec_buf[ADS_FSCAL0] 	= 0x00;		// ADS_FSCAL0
	rec_buf[ADS_FSCAL1] 	= 0x40;		// ADS_FSCAL1
	rec_buf[ADS_GPIODAT] 	= 0x00;		// ADS_GPIODAT
	rec_buf[ADS_GPIOCON] 	= 0x00;		// ADS_GPIOCON


	gpio_set_value(SPI0_CS0,0);
	byteOut(CMD_WREG|ADS_INPMUX);
	byteOut(6);//16
	for (cnt=0;cnt<6;cnt++){//16
		byteOut(rec_buf[cnt+ADS_INPMUX]);// Start writing from ADS_INPMUX register
	}
	gpio_set_value(SPI0_CS0,1);
	
	//readAllADCRegisters();
	
}
int16_t readADCData(uint8_t chNo){
	uint8_t data_buf[5];//,cnt;
	int16_t adcData=0;

	gpio_set_value(SPI0_CS0,0);
	byteOut(CMD_WREG|ADS_INPMUX);
	byteOut(1);
	byteOut(chNo);// Start writing from ADS_INPMUX register
	gpio_set_value(SPI0_CS0,1);
	mdelay(60);

	//readAllADCRegisters();
	gpio_set_value(SPI0_CS0,0);
	byteOut(CMD_STOP);
	gpio_set_value(SPI0_CS0,1);
	gpio_set_value(SPI0_CS0,0);
	byteOut(CMD_START);
	gpio_set_value(SPI0_CS0,1);
	// Take 100 readings
//	for (cnt=0;cnt<10;cnt++){
	mdelay(60);	//Table 7. Data Conversion Time pg 34
/*		do{
		status = gpio_get_value(SPI0_MISO);
		printk(KERN_INFO "ADC Driver: MISO: %02x\n",status);
		} while (status==1);
*/
/*		do{
			gpio_set_value(SPI0_CS0,0);
			byteOut(CMD_RREG|ADS_STATUS);
			byteOut(0x01);
			recvbyte = getByte();
			gpio_set_value(SPI0_CS0,1);
			//printk(KERN_INFO "ADC Driver: ADS_STATUS: %02x\n",recvbyte);
		} while ((recvbyte&0x40)==0x40);
*/
		gpio_set_value(SPI0_CS0,0);
		byteOut(CMD_RDATA);
		data_buf[1] = getByte();
		data_buf[0] = getByte();
		gpio_set_value(SPI0_CS0,1);
		
		adcData = (data_buf[1] << 8) | data_buf[0];
//		printk(KERN_INFO "ADC Driver: Restul: %02x %02x %04x %d\n",data_buf[1],data_buf[0],adc_buf[adcChannel][0],adc_buf[adcChannel][0]);
//	}
	gpio_set_value(SPI0_CS0,0);
	byteOut(CMD_STOP);
	gpio_set_value(SPI0_CS0,1);
	//printk(KERN_INFO "ADC Driver: Restul: %02x %02x %04x %d\n",data_buf[1],data_buf[0],adc_buf[adcChannel][0],adc_buf[adcChannel][0]);
	
	return adcData;
}
void readADCID(void){
	uint8_t recvbyte;//, ascByte[2];
	gpio_set_value(SPI0_CS0,0);
	byteOut(CMD_RESET);
	gpio_set_value(SPI0_CS0,1);
	//mdelay(100);
	// mdelay avoided by reading status register to check RDY bit
	gpio_set_value(SPI0_CS0,0);
	byteOut(CMD_RREG|ADS_STATUS);
	byteOut(0x01);
	recvbyte = getByte();
	gpio_set_value(SPI0_CS0,1);
	printk(KERN_INFO "ADC Driver: ADS_STATUS: %02x\n",recvbyte);

	gpio_set_value(SPI0_CS0,0);
	byteOut(CMD_WREG|ADS_STATUS);
	byteOut(0x01);
	byteOut(0x40);
	gpio_set_value(SPI0_CS0,1);
	printk(KERN_INFO "ADC Driver: Write ADS_STATUS: %02x\n",0x40);

	do{
	gpio_set_value(SPI0_CS0,0);
	byteOut(CMD_RREG|ADS_STATUS);
	byteOut(0x01);
	recvbyte = getByte();
	gpio_set_value(SPI0_CS0,1);
	printk(KERN_INFO "ADC Driver: ADS_STATUS: %02x\n",recvbyte);
	} while ((recvbyte&0x40)==0x40);

	gpio_set_value(SPI0_CS0,0);
	byteOut(CMD_RREG|ADS_ID);
	byteOut(0x01);
	recvbyte = getByte();
	gpio_set_value(SPI0_CS0,1);
	printk(KERN_INFO "ADC Driver: ADS_ID: %02x\n",recvbyte);
}

void readAllADCRegisters(void){
	uint8_t cnt=0;
	
	gpio_set_value(SPI0_CS0,0);
	byteOut(CMD_RREG|ADS_ID);
	byteOut(8);//18
	for (cnt=0;cnt<8;cnt++){//18
		rec_buf[cnt] = getByte();
	}
	gpio_set_value(SPI0_CS0,1);
	
	printk(KERN_INFO "ADC Driver: ====================================\n");
	printk(KERN_INFO "ADC Driver: ADS_ID: %02x\n",rec_buf[0]);
	printk(KERN_INFO "ADC Driver: ADS_STATUS: %02x\n",rec_buf[1]);
	printk(KERN_INFO "ADC Driver: ADS_INPMUX: %02x\n",rec_buf[2]);
	printk(KERN_INFO "ADC Driver: ADS_PGA: %02x\n",rec_buf[3]);
	printk(KERN_INFO "ADC Driver: ADS_DATARATE: %02x\n",rec_buf[4]);
	printk(KERN_INFO "ADC Driver: ADS_REF: %02x\n",rec_buf[5]);
	printk(KERN_INFO "ADC Driver: ADS_IDACMAG: %02x\n",rec_buf[6]);
	printk(KERN_INFO "ADC Driver: ADS_IDACMUX: %02x\n",rec_buf[7]);
	printk(KERN_INFO "ADC Driver: ADS_VBIAS: %02x\n",rec_buf[8]);
	printk(KERN_INFO "ADC Driver: ADS_SYS: %02x\n",rec_buf[9]);
	printk(KERN_INFO "ADC Driver: ADS_RESERVED0: %02x\n",rec_buf[10]);
	printk(KERN_INFO "ADC Driver: ADS_OFCAL0: %02x\n",rec_buf[11]);
	printk(KERN_INFO "ADC Driver: ADS_OFCAL1: %02x\n",rec_buf[12]);
	printk(KERN_INFO "ADC Driver: ADS_RESERVED1: %02x\n",rec_buf[13]);
	printk(KERN_INFO "ADC Driver: ADS_FSCAL0: %02x\n",rec_buf[14]);
	printk(KERN_INFO "ADC Driver: ADS_FSCAL1: %02x\n",rec_buf[15]);
	printk(KERN_INFO "ADC Driver: ADS_GPIODAT: %02x\n",rec_buf[16]);
	printk(KERN_INFO "ADC Driver: ADS_GPIOCON: %02x\n",rec_buf[17]);
	printk(KERN_INFO "ADC Driver: ====================================\n");
	
}
void byteOut(uint8_t byte) {
	uint8_t i;
	for (i = 0; i<8; ++i) {
		if (byte & 0x80) {
			gpio_set_value(SPI0_MOSI,1);
		} else {
			gpio_set_value(SPI0_MOSI,0);
		}
		gpio_set_value(SPI0_CLK,1);
		gpio_set_value(SPI0_CLK,0);
		byte <<= 1;
	}
}

uint8_t syncByte(uint8_t byte) {
	uint8_t byteIn = 0x00;
	uint8_t i;
	for (i = 0; i<8; ++i) {
		if (byte & 0x80) {
			gpio_set_value(SPI0_MOSI,1);
		}
		else{
			gpio_set_value(SPI0_MOSI,0);
		}
		gpio_set_value(SPI0_CLK,1);
		gpio_set_value(SPI0_CLK,0);
		byteIn = (byteIn << 1 ) | gpio_get_value(SPI0_MISO);
		byte <<= 1;
	}
	return byteIn;
}
uint8_t getByte(void) {
	uint8_t byte = 0x00;
	uint8_t i;
	gpio_set_value(SPI0_MOSI,0);
	for (i = 0; i<8; ++i) {
		gpio_set_value(SPI0_CLK,1);
		gpio_set_value(SPI0_CLK,0);
		byte = (byte << 1 ) | gpio_get_value(SPI0_MISO);
	}
	return byte;
}



/// This next calls are  mandatory -- they identify the initialization function
/// and the cleanup function (as above).
module_init(adc_init);
module_exit(adc_exit);
