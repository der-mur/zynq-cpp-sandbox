# zynq-cpp-sandbox
This is my sandbox for exploring the use of C++ to develop projects for the AMD (Xilinx) Zynq.
Points:
- Main aim is to begin the process of creating a range of c++ drivers which can be re-used for other Zynq-7000/Ultrascale/Microblaze designs. 
- Simple 'skeleton' drivers, not fully functional, not fully tested.
- A lot of the underlying C code is still being used. For example:
  - The ```xparameters.h file``` is used for accessing system paramters and settings.
  - The Xilinx interrupt driver code has not been rewritten as c++ code, etc.
- Very little defensive code is used in the drivers as they curently stand.
- Having an understanding of Zynq HW, IP blocks and c-code drivers would be useful in the discussion below.

## Drivers
### AXI_GPIO (Programmable Logic GPIO IP)
[```axi_gpio.cpp```, ```axi_gpio.h```](2023.2/zybo-z7-20/hw_proj1/vitis_classic/sw_proj5_cpp/src/classes)

![AXI GPIO Class Diagram](assets/images/axigpio_class_diagram_v2.png)

The AxiGpio class is very simple, but it shows some basic principles that are repeated in other classes. For example, pointers to each register in the driver are included in the private data of the class. (Note that the interrupt functionality is not included in this first version. Also note that using pointers for each register might not be a great idea on resource-constrained uControllers or for drivers with a large register set, but it is fine for this exploratory work on the Zynq i.e. small drivers on memory-rich devices.)
```c++
  private:
	device_reg *p_CH1_DATA_REG { nullptr };
	device_reg *p_CH1_TRI_REG { nullptr };
	device_reg *p_CH2_DATA_REG { nullptr };
	device_reg *p_CH2_TRI_REG { nullptr };
```
The ```device_reg``` is an alias for a volatile uint32_t data type:
```c++
using device_reg = std::uint32_t volatile;
```

When the constructor for the device is called, the private data pointers are set to the correct HW value using the ```reinterpret_cast``` operation.

```c++
AxiGpio::AxiGpio(std::uint32_t base)
{
	p_CH1_DATA_REG = reinterpret_cast<device_reg*>(base + offset::gpio::CH1_DATA);
	p_CH1_TRI_REG = reinterpret_cast<device_reg*>(base + offset::gpio::CH1_TRI);
	p_CH2_DATA_REG = reinterpret_cast<device_reg*>(base + offset::gpio::CH2_DATA);
	p_CH2_TRI_REG = reinterpret_cast<device_reg*>(base + offset::gpio::CH2_TRI);
 }
```

When creating the AXI GPIO object, the base address must be passed to the contructor. This is not necessarily the case for the other drivers discussed below, but the AXI IP functionality is very dependent on the users programmable logic design in that multiple blocks can be created and the base addresses can be changed. To accomodate this uncertainty I also have a file called ```axi_reg.h``` which the user can modify as needed. (```xparameters.h``` is still used to get the system info i.e. ```XPAR_GPIO_0_BASEADDR``` in this case.) The register offsets are also included in ```axi_reg.h```.
```c++
namespace axi
{
  namespace reg
  {
  	  namespace base_addr
	  {
      	  constexpr std::uint32_t gpio0			= std::uint32_t(XPAR_GPIO_0_BASEADDR);
      	  constexpr std::uint32_t spi0			= std::uint32_t(XPAR_SPI_0_BASEADDR);
	  } // base_addr



  	  // REGISTER OFFSETS
  	  namespace offset
	  {
  	  	  // GPIO OFFSETS
  	  	  namespace gpio
		  {
			constexpr std::uint16_t CH1_DATA		= 0;
			constexpr std::uint16_t CH1_TRI			= 0x0004;
			constexpr std::uint16_t CH2_DATA		= 0x0008;
			constexpr std::uint16_t CH2_TRI			= 0x000C;

			constexpr std::uint16_t GBL_INTR_EN		= 0x011C;
			constexpr std::uint16_t INTR_STATUS		= 0x0120;
			constexpr std::uint16_t INTR_ENABLE		= 0x0128;
		  }  // gpio
  	  } // offset


  } // reg
} // axi
```
<br/><br/>

The AXI GPIO object is created in ```system_config.cpp```. It's a static object (i.e. file-scoped to ```system_config.cpp```), and a pointer is also created in the ```sys_init()``` function so that other code can access the GPIO functions. (That is, the pointer can be passed around, giving some control over who has access to the AXIO GPIO block.)
```c++
// system_config.cpp: Call constructor to create the AXI GPIO object
static AxiGpio	AxiGpio0(axi::reg::base_addr::gpio0);

sys_init() {
...
// Create pointer to the AXI_GPIO object: 
AxiGpio* p_AxiGpio0 = &AxiGpio0;
...
}
```

To accomodate flexibility at the board level, I have a namespace called ```board::gpio::pin_names::axi``` in a file called ```settings.h```. This maps out the GPIO connections at the board level:
```c++
namespace board
{
	namespace gpio
	{
		namespace pin_names
		{
			namespace axi
			{
				// CHANNEL 1 (ALL OUTPUTS)
				constexpr std::uint8_t LED0				= 0;
				constexpr std::uint8_t LED1				= 1;
				constexpr std::uint8_t LED2				= 2;
				constexpr std::uint8_t LED3				= 3;
				constexpr std::uint8_t PMOD_JE_PIN1		= 4;
				constexpr std::uint8_t PMOD_JE_PIN2		= 5;
				constexpr std::uint8_t PMOD_JE_PIN3		= 6;
				constexpr std::uint8_t PMOD_JE_PIN4		= 7;

				// CHANNEL 2 (ALL INPUTS)
				constexpr std::uint8_t BTN0				= 0;
				constexpr std::uint8_t BTN1				= 1;
				constexpr std::uint8_t BTN2				= 2;
				constexpr std::uint8_t BTN3				= 3;
				constexpr std::uint8_t SW0				= 4;
				constexpr std::uint8_t SW1				= 5;
				constexpr std::uint8_t SW2				= 6;
				constexpr std::uint8_t SW3				= 7;
				constexpr std::uint8_t PMOD_JE_PIN7		= 8;
				constexpr std::uint8_t PMOD_JE_PIN8		= 9;
				constexpr std::uint8_t PMOD_JE_PIN9		= 10;
				constexpr std::uint8_t PMOD_JE_PIN10	= 11;
			} // axi
		} // pin_names
	} // gpio
} // board
```
I also have a file called ```common.h``` which contains entities that might be used for different modules (i.e. AXI GPIO and PS GPIO):
```c++
namespace common
{
	enum class GpioOperation {Set, Clear, Toggle};
	enum class BankType {mio, emio};
	enum class Direction {Input, Output};
}
```

Finally, here are some examples of how to toggle an LED, or set/clear PMOD pins:
```c++
p_AxiGpio->writeCh1Pin(LED1, GpioOperation::Toggle);
p_AxiGpio->writeCh1Pin(PMOD_JE_PIN1, GpioOperation::Set);
p_AxiGpio->writeCh1Pin(PMOD_JE_PIN1, GpioOperation::Clear);
```


### PS7 GPIO
Abc

