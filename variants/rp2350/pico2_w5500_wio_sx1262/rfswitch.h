#pragma once
#include "RadioLib.h"

// Keep LR20x0 naming while RadioLib exposes LR2021 symbols.
#ifndef LR20x0
#define LR20x0 LR2021
#endif

// LR2021 RF switch matrix following the standard Semtech / Seeed T1000-E reference topology.
// DIO5 -> antenna path select (HIGH = sub-GHz LF)
// DIO6 -> TX enable / HP PA select
// DIO7 -> not connected (no GNSS on LR2021)
// DIO8 -> RF front-end power enable

static const uint32_t lr20x0_rfswitch_dio_pins[] = {RADIOLIB_LR2021_DIO5, RADIOLIB_LR2021_DIO6, RADIOLIB_LR2021_DIO7,
                                                    RADIOLIB_LR2021_DIO8, RADIOLIB_NC};

static const Module::RfSwitchMode_t lr20x0_rfswitch_table[] = {
    // clang-format off
    // mode               DIO5  DIO6  DIO7  DIO8
    {LR20x0::MODE_STBY,   {LOW,  LOW,  LOW,  LOW}},
    {LR20x0::MODE_RX,     {HIGH, LOW,  LOW,  HIGH}},
    {LR20x0::MODE_TX,     {HIGH, HIGH, LOW,  HIGH}},
    {LR20x0::MODE_RX_HF,  {LOW,  LOW,  LOW,  LOW}},
    {LR20x0::MODE_TX_HF,  {LOW,  LOW,  LOW,  LOW}},
    END_OF_MODE_TABLE,
    // clang-format on
};
