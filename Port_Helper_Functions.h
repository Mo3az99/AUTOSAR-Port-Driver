#ifndef PORT_HELPER_FUNCTIONS
#define PORT_HELPER_FUNCTIONS
#include "Port.h"


// #define __I volatile const /*!< Defines 'read only' permissions    */
// #define __O volatile       /*!< Defines 'write only' permissions   */
// #define __IO volatile      /*!< Defines 'read / write' permissions */

// TYPEDEF struct
// { /*                              !< GPIO Structure >!                                          */
//     __I uint32 RESERVED[255];
//     __IO uint32 DATA;  /*!< GPIO Data                                                             */
//     __IO uint32 DIR;   /*!< GPIO Direction                                                        */
//     __IO uint32 IS;    /*!< GPIO Interrupt Sense                                                  */
//     __IO uint32 IBE;   /*!< GPIO Interrupt Both Edges                                             */
//     __IO uint32 IEV;   /*!< GPIO Interrupt Event                                                  */
//     __IO uint32 IM;    /*!< GPIO Interrupt Mask                                                   */
//     __IO uint32 RIS;   /*!< GPIO Raw Interrupt Status                                             */
//     __IO uint32 MIS;   /*!< GPIO Masked Interrupt Status                                          */
//     __O uint32 ICR;    /*!< GPIO Interrupt Clear                                                  */
//     __IO uint32 AFSEL; /*!< GPIO Alternate Function Select                                        */
//     __I uint32 RESERVED1[55];
//     __IO uint32 DR2R;   /*!< GPIO 2-mA Drive Select                                                */
//     __IO uint32 DR4R;   /*!< GPIO 4-mA Drive Select                                                */
//     __IO uint32 DR8R;   /*!< GPIO 8-mA Drive Select                                                */
//     __IO uint32 ODR;    /*!< GPIO Open Drain Select                                                */
//     __IO uint32 PUR;    /*!< GPIO Pull-Up Select                                                   */
//     __IO uint32 PDR;    /*!< GPIO Pull-Down Select                                                 */
//     __IO uint32 SLR;    /*!< GPIO Slew Rate Control Select                                         */
//     __IO uint32 DEN;    /*!< GPIO Digital Enable                                                   */
//     __IO uint32 LOCK;   /*!< GPIO Lock                                                             */
//     __IO uint32 CR;     /*!< GPIO Commit                                                           */
//     __IO uint32 AMSEL;  /*!< GPIO Analog Mode Select                                               */
//     __IO uint32 PCTL;   /*!< GPIO Port Control                                                     */
//     __IO uint32 ADCCTL; /*!< GPIO ADC Control                                                      */
//     __IO uint32 DMACTL; /*!< GPIO DMA Control                                                      */
// } GPIO_Type;

// STATIC INLINE void CHECK_LOCK_COMMIT(GPIO_Type *, Port_PinType, Port_PinType);
// STATIC INLINE void SETDIRECTION(GPIO_Type *, Port_PinType, Port_PinDirectionType);
// STATIC INLINE void SETINTERNALRESISTOR(GPIO_Type *, Port_PinType, Port_PinInternalResistorType);
// STATIC INLINE void SETLEVEL(GPIO_Type *, Port_PinType, Port_PinValueType);

#endif /* PORT_HELPER_FUNCTIONS */