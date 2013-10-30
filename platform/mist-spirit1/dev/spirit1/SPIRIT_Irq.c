/**
 * @file    SPIRIT_Irq.c
 * @author  MSH RF/ART Team IMS-Systems Lab
 * @version V2.0.1
 * @date    November 8, 2011
 * @brief   Configuration and management of SPIRIT IRQs.
 * @details
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOURCE CODE IS PROTECTED BY A LICENSE.
 * FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
 * IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
 *
 * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
 */


/* Includes ------------------------------------------------------------------*/
#include "SPIRIT_Irq.h"
#include "SPIRIT_Spi_Driver.h"



/**
 * @addtogroup SPIRIT_Libraries
 * @{
 */


/**
 * @addtogroup SPIRIT_Irq
 * @{
 */


/**
 * @defgroup Irq_Private_TypesDefinitions       IRQ Private Types Definitions
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup Irq_Private_Defines                IRQ Private Defines
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup Irq_Private_Macros                 IRQ Private Macros
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup Irq_Private_Variables              IRQ Private Variables
 * @{
 */


/**
 *@}
 */


/**
 * @defgroup Irq_Private_FunctionPrototypes     IRQ Private Function Prototypes
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup Irq_Private_Functions              IRQ Private Functions
 * @{
 */


/**
 * @brief  De initializate the SpiritIrqs structure setting all the bitfield to 0.
 *         Moreover, it sets the IRQ mask registers to 0x00000000, disabling all IRQs.
 * @param  pxIrqInit pointer to a variable of type @ref SpiritIrqs, in which all the
 *         bitfields will be settled to zero.
 * @retval None.
 */
void SpiritIrqDeInit(SpiritIrqs* pxIrqInit)
{
  uint32_t tempValue = 0x00000000;
  uint8_t tempRegValue[4]={0x00,0x00,0x00,0x00};

  if(pxIrqInit!=NULL)
  {
    /* Sets the bitfields of passed structure to one */
    *pxIrqInit = (*(SpiritIrqs*)&tempValue);
  }

  /* Writes the IRQ_MASK registers */
  g_xStatus = SpiritSpiWriteRegisters(IRQ_MASK3_BASE, 4, tempRegValue);
}


/**
 * @brief  Enables all the IRQs according to the user defined pxIrqInit structure.
 * @param  pxIrqInit pointer to a variable of type @ref SpiritIrqs, through which the
 *         user enable specific IRQs. This parameter is a pointer to a SpiritIrqs.
 *         For example suppose to enable only the two IRQ Low Battery Level and Tx Data Sent:
 * @code
 * SpiritIrqs myIrqInit = {0};
 * myIrqInit.IRQ_LOW_BATT_LVL = 1;
 * myIrqInit.IRQ_TX_DATA_SENT = 1;
 * SpiritIrqInit(&myIrqInit);
 * @endcode
 * @retval None.
 */
void SpiritIrqInit(SpiritIrqs* pxIrqInit)
{
  uint8_t tempRegValue[4];
  uint8_t* tmpPoint;
  char i;

  /* Cast the bitfields structure in an array of char using */
  tmpPoint = (uint8_t*)(pxIrqInit);
  for(i=0; i<4; i++)
  {
    tempRegValue[3-i]= tmpPoint[i];
  }

  /* Writes the IRQ_MASK registers */
  g_xStatus = SpiritSpiWriteRegisters(IRQ_MASK3_BASE, 4, tempRegValue);

}


/**
 * @brief  Enables or disables a specific IRQ.
 * @param  xIrq IRQ to enable or disable.
 *         This parameter can be any value of @ref IrqList.
 * @param  xNewState new state for the IRQ.
 *         This parameter can be: S_ENABLE or S_DISABLE.
 * @retval None.
 */
void SpiritIrq(IrqList xIrq, SpiritFunctionalState xNewState)
{
  uint8_t tempRegValue[4];
  uint32_t tempValue = 0;
  char i, j;

  /* Check the parameters */
  s_assert_param(IS_SPIRIT_IRQ_LIST(xIrq));
  s_assert_param(IS_SPIRIT_FUNCTIONAL_STATE(xNewState));

  /* Reads the IRQ_MASK registers */
  g_xStatus = SpiritSpiReadRegisters(IRQ_MASK3_BASE, 4, tempRegValue);

  /* Build the IRQ mask word */
  for(i=0; i<4; i++)
    tempValue += ((uint32_t)tempRegValue[i])<<(8*(3-i));

  /* Rebuild the new mask according to user request */
  if(xNewState == S_DISABLE)
    tempValue &= (~xIrq);
  else
    tempValue |= (xIrq);

  /* Build the array of bytes to write in the IRQ_MASK registers */
  for(j=0; j<4; j++)
    tempRegValue[j] = (uint8_t)(tempValue>>(8*(3-j)));

  /* Writes the new IRQ mask in the corresponding registers */
  g_xStatus = SpiritSpiWriteRegisters(IRQ_MASK3_BASE, 4, tempRegValue);

}


/**
 * @brief  Fills a pointer to a structure of SpiritIrqs type reading the IRQ_MASK registers.
 * @param  pxIrqMask pointer to a variable of type @ref SpiritIrqs, through which the
 *         user can read which IRQs are enabled. All the bitfields equals to zero correspond
 *         to enabled IRQs, while all the bitfields equals to one correspond to disabled IRQs.
 *         This parameter is a pointer to a SpiritIrqs.
 *         For example suppose that the Power On Reset and RX Data ready are the only enabled IRQs.
 * @code
 * SpiritIrqs myIrqMask;
 * SpiritIrqGetStatus(&myIrqMask);
 * @endcode
 * Then
 * myIrqMask.IRQ_POR and myIrqMask.IRQ_RX_DATA_READY are equal to 0
 * while all the other bitfields are equal to one.
 * @retval None.
 */
void SpiritIrqGetMask(SpiritIrqs* pxIrqMask)
{
  uint8_t tempRegValue[4];
  uint32_t tempValue=0;
  char i;

  /* Reads IRQ_MASK registers */
  g_xStatus = SpiritSpiReadRegisters(IRQ_MASK3_BASE, 4, tempRegValue);

  /* Build the IRQ mask word */
  for(i=0; i<4; i++)
    tempValue += ((uint32_t)tempRegValue[i])<<(8*(3-i));

  /* Fill the user IrqMask structure */
  *pxIrqMask = *((SpiritIrqs*)&tempValue);

}


/**
 * @brief  Filla a pointer to a structure of SpiritIrqs type reading the IRQ_STATUS registers.
 * @param  pxIrqStatus pointer to a variable of type @ref SpiritIrqs, through which the
 *         user can read the status of all the IRQs. All the bitfields equals to one correspond
 *         to the raised interrupts. This parameter is a pointer to a SpiritIrqs.
 *         For example suppose that the XO settling timeout is raised as well as the Sync word
 *         detection.
 * @code
 * SpiritIrqs myIrqStatus;
 * SpiritIrqGetStatus(&myIrqStatus);
 * @endcode
 * Then
 * myIrqStatus.IRQ_XO_COUNT_EXPIRED and myIrqStatus.IRQ_VALID_SYNC are equals to 1
 * while all the other bitfields are equals to zero.
 * @retval None.
 */
void SpiritIrqGetStatus(SpiritIrqs* pxIrqStatus)
{
  uint8_t tempRegValue[4];
  uint32_t tempValue = 0;
  uint8_t i;

  /* Reads IRQ_STATUS registers */
  g_xStatus = SpiritSpiReadRegisters(IRQ_STATUS3_BASE, 4, tempRegValue);

  /* Build the IRQ Status word */
  for(i=0; i<4; i++)
    tempValue += ((uint32_t)tempRegValue[i])<<(8*(3-i));

  /* Fill the user IrqStatus structure */
  *pxIrqStatus = *((SpiritIrqs*)&tempValue);

}


/**
 * @brief  Clear the IRQ status registers.
 * @param  None.
 * @retval None.
 */
void SpiritIrqClearStatus(void)
{
  uint8_t tempRegValue[4];

  /* Reads the IRQ_STATUS registers clearing all the flags */
  g_xStatus = SpiritSpiReadRegisters(IRQ_STATUS3_BASE, 4, tempRegValue);

}


/**
 * @brief  Verifies if a specific IRQ has been generated.
 *         The call resets all the IRQ status, so it can't be used in case of multiple raising interrupts.
 * @param  xFlag IRQ flag to be checked.
 *         This parameter can be any value of @ref IrqList.
 * @retval SpiritBool S_TRUE or S_FALSE.
 */
SpiritBool SpiritIrqCheckFlag(IrqList xFlag)
{
  uint8_t tempRegValue[4];
  uint32_t tempValue = 0;
  uint8_t i;
  SpiritBool flag;

  /* Check the parameters */
  s_assert_param(IS_SPIRIT_IRQ_LIST(xFlag));

  /* Reads registers and build the status word */
  g_xStatus = SpiritSpiReadRegisters(IRQ_STATUS3_BASE, 4, tempRegValue);
  for(i=0; i<4; i++)
    tempValue += ((uint32_t)tempRegValue[i])<<(8*(3-i));

  if(tempValue & xFlag)
    flag = S_TRUE;
  else
    flag = S_FALSE;

  return flag;

}


/**
 *@}
 */


/**
 *@}
 */


/**
 *@}
 */




/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
