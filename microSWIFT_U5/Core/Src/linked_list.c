/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : linked_list.c
  * Description        : This file provides code for the configuration
  *                      of the LinkedList.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "linked_list.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

DMA_NodeTypeDef GNSS_LL_Node;
DMA_QListTypeDef GNSS_LL_Queue;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/**
  * @brief  DMA Linked-list YourQueueName configuration
  * @param  None
  * @retval None
  */
HAL_StatusTypeDef MX_GNSS_LL_Queue_Config(void)
{
  HAL_StatusTypeDef ret = HAL_OK;
  /* DMA node configuration declaration */
  DMA_NodeConfTypeDef pNodeConfig;
//  DMA_HandleTypeDef handle_GPDMA1_Channel0 = handle_GPDMA1_Channel0;


  /* Set node configuration ################################################*/
  pNodeConfig.NodeType = DMA_GPDMA_LINEAR_NODE;
  pNodeConfig.Init.Request = GPDMA1_REQUEST_LPUART1_RX;
  pNodeConfig.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  pNodeConfig.Init.Direction = DMA_PERIPH_TO_MEMORY;
  pNodeConfig.Init.SrcInc = DMA_SINC_FIXED;
  pNodeConfig.Init.DestInc = DMA_DINC_INCREMENTED;
  pNodeConfig.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
  pNodeConfig.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
  pNodeConfig.Init.SrcBurstLength = 1;
  pNodeConfig.Init.DestBurstLength = 1;
  pNodeConfig.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0;
  pNodeConfig.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
//  pNodeConfig.RepeatBlockConfig.RepeatCount = 1;
//  pNodeConfig.RepeatBlockConfig.SrcAddrOffset = 0;
//  pNodeConfig.RepeatBlockConfig.DestAddrOffset = 0;
//  pNodeConfig.RepeatBlockConfig.BlkSrcAddrOffset = 0;
//  pNodeConfig.RepeatBlockConfig.BlkDestAddrOffset = 0;
  pNodeConfig.TriggerConfig.TriggerPolarity = DMA_TRIG_POLARITY_MASKED;
  pNodeConfig.DataHandlingConfig.DataExchange = DMA_EXCHANGE_NONE;
  pNodeConfig.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;
  pNodeConfig.SrcAddress = 0;
  pNodeConfig.DstAddress = 0;
  pNodeConfig.DataSize = 100;

//  handle_GPDMA1_Channel0.InitLinkedList.LinkedListMode = DMA_LINKEDLIST_CIRCULAR;

  /* Build GNSS_LL_Node Node */
  ret |= HAL_DMAEx_List_BuildNode(&pNodeConfig, &GNSS_LL_Node);

  /* Insert GNSS_LL_Node to Queue */
  ret |= HAL_DMAEx_List_InsertNode_Tail(&GNSS_LL_Queue, &GNSS_LL_Node);

  ret |= HAL_DMAEx_List_SetCircularMode(&GNSS_LL_Queue);

  return ret;
}

