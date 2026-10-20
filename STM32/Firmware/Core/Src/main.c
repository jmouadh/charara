/**
  ******************************************************************************
  * @file    main.c
  * @brief   Application entry point - intentionally minimal.
  * @note    Board bring-up lives in App/Src/robot_board.c; all robot logic
  *          (protocol, motion, display, status) lives in the App layer and is
  *          executed by ROBOT_APP_run() (bare metal) or by FreeRTOS tasks
  *          (see Docs/02-firmware-architecture.md).
  *          If the CubeMX .ioc is regenerated, keep this file minimal and
  *          re-apply the bring-up as documented in Docs/02.
  * @author  Mouadh Jeridi <jeridimouadh@gmail.com>
  * @version 3.0.0
  * @date    2026-09-28
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "robot_app.h"

/* Main function -------------------------------------------------------------*/
/**
  * @brief  The application entry point.
  * @retval int (never reached: the scheduler loops forever)
  */
int main(void)
{
  ROBOT_APP_init();
  ROBOT_APP_run();

  return 0;
}
