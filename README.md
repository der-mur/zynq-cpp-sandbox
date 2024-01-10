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
[```ps_gpio.cpp```, ```ps_gpio.h```](2023.2/zybo-z7-20/hw_proj1/vitis_classic/sw_proj5_cpp/src/classes)

The GPIO block internal to the Zynq APU is interesting as it is part of the very flexible MIO system. MIO stands for multiplexed IO, and it means that a range of peripherals (I2C, UART, SPI, GPIO, etc) can be mapped to the limited pinout on the processing side of the Zynq. The GPIO block for the Zynq-7000 is quite large, having four banks comprising a total of 118 pins, but usually only a small range of pins will be left over once the other peripherals are allocated. The upper two banks can however be routed through the programmable logic to available pins in that section. The basic layout is shown below. (Note that there are six banks in Ultrascale devices: 3 MIO and 3 EMIO.)

![PS7 GPIO Block Diagram](assets/images/ps_gpio1.png)
<br/><br/>

The MIO configuration for the current project is shown below. The LED and two switches are fixed at board layout time. The PMOD header is more flexible as other peripherals can use these pins after the board is created; however, in this project we simply use them as GPIO pins.

![PS7 GPIO Block Diagram](assets/images/ps_gpio2.png)
<br/><br/>

The PS GPIO driver class diagram is shown below. In fact two classes are involved, and a PsGpio object is created using the OOP principle of composition: PsGpioBank represents an individual bank, and PsGpio is composed of an array of four banks. (In an Ultrascale device, PsGpio would be made up of six PsGpioBank's.) 

![PS GPIO Class Diagram](assets/images/psgpio_class_diagram.png)
<br/><br/>

The constructor for a single bank is as follows:
```c++
PsGpioBank::PsGpioBank(std::uint16_t bank_number)
{
	// General settings
	m_bank_number = bank_number;
	m_bank_addr = bank_base_addr[bank_number];
	m_bank_type = bank_type[bank_number];
	m_bank_size_hex = bank_size_hex[bank_number];
	m_bank_size_offset = bank_size_offsets[bank_number];
	m_bank_gpio_allowed = bank_gpio_allowed[bank_number];

	// Register pointers
	p_MASK_DATA_LSW_REG = reinterpret_cast<device_reg*>(m_bank_addr + MASK_DATA_LSW);
	p_MASK_DATA_MSW_REG = reinterpret_cast<device_reg*>(m_bank_addr + MASK_DATA_MSW);
	p_DATA_WRITE_REG = reinterpret_cast<device_reg*>(m_bank_addr + DATA_WRITE - (m_bank_number*0x4));
	p_DATA_READ_REG = reinterpret_cast<device_reg*>(m_bank_addr + DATA_READ - (m_bank_number*0x4));

	p_DIR_MODE_REG = reinterpret_cast<device_reg*>(m_bank_addr + DIR_MODE + (m_bank_number*0x38));
	p_OUTPUT_ENABLE_REG = reinterpret_cast<device_reg*>(m_bank_addr + OUTPUT_ENABLE + (m_bank_number*0x38));
	p_INTR_MASK_REG = reinterpret_cast<device_reg*>(m_bank_addr + INTR_MASK + (m_bank_number*0x38));
	p_INTR_ENABLE_UNMASK_REG = reinterpret_cast<device_reg*>(m_bank_addr + INTR_ENABLE_UNMASK + (m_bank_number*0x38));
	p_INTR_DISABLE_MASK_REG = reinterpret_cast<device_reg*>(m_bank_addr + INTR_DISABLE_MASK + (m_bank_number*0x38));
	p_INTR_STATUS_REG = reinterpret_cast<device_reg*>(m_bank_addr + INTR_STATUS + (m_bank_number*0x38));
	p_INTR_TYPE_REG = reinterpret_cast<device_reg*>(m_bank_addr + INTR_TYPE + (m_bank_number*0x38));
	p_INTR_POLARITY_REG = reinterpret_cast<device_reg*>(m_bank_addr + INTR_POLARITY + (m_bank_number*0x38));
	p_INTR_ANY_EDGE_SENSE_REG = reinterpret_cast<device_reg*>(m_bank_addr + INTR_ANY_EDGE_SENSE + (m_bank_number*0x38));
}
```

The constructor for the PsGpio object, then is as follows:

```c++
PsGpio::PsGpio(){
	for(int i=0; i <= n_banks; i++)
	{
		PsGpioBankArray[i] = PsGpioBank(i);
	}
}
```
The PsGpio object is created in ```system_config.cpp``` (again it is static, and a pointer is created in ```sys_init()``` to allow access to the GPIO functionality):
```c++
static PsGpio	PsGpio0;
```
The parameters required for object creation can be found in ```settings.h``` (see also ```common.h``` for the BankType definition):
```c++
namespace sys
{
	namespace ps_gpio
	{
		constexpr std::uint8_t n_banks				= 4; // Four GPIO banks in Zynq7000

		// Bank 0: MIO; Bank 1: MIO; Bank 2: EMIO; Bank 3: EMIO;
		constexpr BankType bank_type[n_banks]		= { BankType::mio, BankType::mio, BankType::emio, BankType::emio };

		// Bank 0: Pins 0,7,9,10,11,12,13,14,15; Bank 1: 50, 51; Bank 2: All pins; Bank 3: All pins
		constexpr std::uint32_t bank_gpio_allowed[n_banks] = { 0x0000FE81, 0x00060000, 0xFFFFFFFF, 0xFFFFFFFF };

		// Bank size: Bank 0: 32-bits; Bank 1: 22-bits; Bank 2: 32-bits; Bank 3: 32-bits
		constexpr std::uint32_t	bank_size_hex[n_banks] = { 0xFFFFFFFF, 0x003FFFFF, 0xFFFFFFFF, 0xFFFFFFFF };
		constexpr std::uint32_t	bank_size_offsets[n_banks] = { 32, 53, 85, 117 };

		// Bank 0: Pins 0,7,9,10,11,12,13,14,15 set to output.
		constexpr std::uint32_t bank_gpio_output[n_banks] = { 0x0000FE81, 0x00000000, 0x00000000, 0x00000000 };

		constexpr std::uint32_t bank_base_addr[n_banks] = { XPAR_PS7_GPIO_0_BASEADDR,
															XPAR_PS7_GPIO_0_BASEADDR + 0x8,
															XPAR_PS7_GPIO_0_BASEADDR + 0x10,
															XPAR_PS7_GPIO_0_BASEADDR + 0x18 };

	} // ps_gpio
} // sys
```

Finally, the PS GPIO functions can be used in a very similar fashion to the AXI GPIO functions (the interface has been designed so that the signatures are identical):

```c++
p_PsGpio->writePin(LED4, GpioOperation::Toggle);
p_PsGpio->writePin(PMOD_JF_PIN4, GpioOperation::Set);
p_PsGpio->writePin(PMOD_JF_PIN4, GpioOperation::Clear);

```



















