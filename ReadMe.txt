﻿/*******************************************************************************
*
* E M B E D D E D   W I Z A R D   P R O J E C T
*
*                                                Copyright (c) TARA Systems GmbH
*                                    written by Paul Banach and Manfred Schweyer
*
********************************************************************************
*
* This software is delivered "as is" and shows the usage of other software
* components. It is provided as an example software which is intended to be
* modified and extended according to particular requirements.
*
* TARA Systems hereby disclaims all warranties and conditions with regard to the
* software, including all implied warranties and conditions of merchantability
* and non-infringement of any third party IPR or other rights which may result
* from the use or the inability to use the software.
*
********************************************************************************
*
* DESCRIPTION:
*
*   Embedded Wizard Build Environment for the follwing target platform:
*
*   NXP IMXRT1064-EVK
*
*   This package contains everything needed to build and run an Embedded Wizard
*   generated UI application on a dedicated target.
*   This Build Environment for Embedded Wizard generated UI applications was
*   tested by using the following components:
*   - Embedded Wizard Studio V14
*   - Embedded Wizard Platform Package (GFX/RTE) V14.00.00
*   - Embedded Wizard Build Environment V14.00.00.00
*   - NXP IMXRT1064-EVK development board
*   - MCUXpresso SDK V2.16.000
*   - Arm GNU Toolchain V11.2-2022.02
*   - IAR Embedded Workbench 9.60.2, IAR C/C++ Compiler V9.60.2.399/W64 for ARM
*   - Keil MDK-ARM Professional Version 5.41.0.0, ARM Compiler 6.22 (armclang)
*   - MCUXpresso IDE V24.9.25
*
*******************************************************************************/

Getting started with the target system:
---------------------------------------
  To get your first UI application generated by Embedded Wizard working on a
  particular reference platform, we have prepared a detailed article covering
  all necessary steps.
  We highly recommend to study the following document:

  https://doc.embedded-wizard.de/getting-started-mimxrt1064-evk


Getting started with Embedded Wizard Studio:
--------------------------------------------
  In order to get familiar with Embedded Wizard Studio and the UI development
  work-flow, we highly recommend to study our online documentation:

  https://doc.embedded-wizard.de

  Furthermore, we have collected many 'Questions and Answers' covering
  typical Embedded Wizard programming aspects. Please visit our community:

  https://ask.embedded-wizard.de

  Please use this platform to drop your questions, answers and ideas.


Build Environment Release Notes:
--------------------------------
  The Graphics Engine (GFX) and Runtime Environment (RTE) version history is
  included in the Embedded Wizard release notes. Please see our online
  documentation to be informed of any improvements, changes or bug fixes:

  https://doc.embedded-wizard.de/release-notes

  Any improvements, changes or bug fixes related only to the Build Environment
  are described directly in the above mentioned 'Getting Started' article in
  the section 'Release Notes'.


/*******************************************************************************
* Important note:
* This Build Environment is intended to be used as template for Embedded Wizard
* GUI applications on the development board or your customer specific hardware.
* Please take care to adjust all timings and hardware configurations (e.g. system
* clock configurations, memory timings, MPU and cache settings) according to your
* needs and hardware capabilities in order to ensure a stable system.
* Please also take the hardware manufacturer's corresponding specifications,
* application notes and erratas into account.
*******************************************************************************/


Known Issues:
-------------
- Sometimes the redlinkserver tool cannot establish a connection with the board.
  As a result, the console will not work and the flash process will not work or
  stop unexpectedly.
  In order to get the board working please start the MCUXpresso IDE project and
  simply start the GUI Flash Tool at the right end of the tool bar. Then the
  probe discovery will detect the board correctly. After completing this step,
  the console and flash process will work as usual.
  This seems to be an issue under Windows 10.


3rdParty-Components
-------------------
The following 3rdParty-Components are used:

1. NXP SDK: ./ThirdParty/MCUXpressoSDK/SW-Content-Register.txt

2. Gnu Make - GNU General Public License v2 or later

3. Gnu GCC - GNU General Public License v2 or later

4. NXP Redlink flash utility - ./ThirdParty/MCUXpressoIDE/SoftwareContentRegister.txt
