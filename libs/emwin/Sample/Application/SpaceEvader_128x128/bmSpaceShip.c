/*********************************************************************
*                SEGGER Microcontroller GmbH                         *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2018  SEGGER Microcontroller GmbH                *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

** emWin V5.48 - Graphical user interface for embedded applications **
All  Intellectual Property rights  in the Software belongs to  SEGGER.
emWin is protected by  international copyright laws.  Knowledge of the
source code may not be used to write a similar product.  This file may
only be used in accordance with the following terms:

The software  has been licensed to  Cypress Semiconductor Corporation,
whose registered  office is situated  at 198 Champion Ct. San Jose, CA 
95134 USA  solely for the  purposes of creating  libraries for Cypress
PSoC3 and  PSoC5 processor-based devices,  sublicensed and distributed
under  the  terms  and  conditions  of  the  Cypress  End User License
Agreement.
Full source code is available at: www.segger.com

We appreciate your understanding and fairness.
----------------------------------------------------------------------
Licensing information
Licensor:                 SEGGER Microcontroller Systems LLC
Licensed to:              Cypress Semiconductor Corp, 198 Champion Ct., San Jose, CA 95134, USA
Licensed SEGGER software: emWin
License number:           GUI-00319
License model:            Services and License Agreement, signed June 10th, 2009
Licensed platform:        Any Cypress platform (Initial targets are: PSoC3, PSoC5)
----------------------------------------------------------------------
Support and Update Agreement (SUA)
SUA period:               2009-06-12 - 2022-07-27
Contact to extend SUA:    sales@segger.com
----------------------------------------------------------------------
File        : bmSpaceShip.c
Purpose     : 128 * 113 pixels, 2 colors, Has transparency
---------------------------END-OF-HEADER------------------------------
*/

#include <stdlib.h>

#include "GUI.h"

#ifndef GUI_CONST_STORAGE
  #define GUI_CONST_STORAGE const
#endif

extern GUI_CONST_STORAGE GUI_BITMAP bmSpaceShip;

/*********************************************************************
*
*       Palette
*
*  Description
*    The following are the entries of the palette table.
*    The entries are stored as a 32-bit values of which 24 bits are
*    actually used according to the following bit mask: 0xBBGGRR
*
*    The lower   8 bits represent the Red   component.
*    The middle  8 bits represent the Green component.
*    The highest 8 bits represent the Blue  component.
*/
static GUI_CONST_STORAGE GUI_COLOR _ColorsSpaceShip[] = {
  0x000000, 0xFFFFFF
};

static GUI_CONST_STORAGE GUI_LOGPALETTE _PalSpaceShip = {
  2,  // Number of entries
  0,  // No transparency
  &_ColorsSpaceShip[0]
};

static GUI_CONST_STORAGE unsigned char _acSpaceShip[] = {
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXX_, _XXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXX__, __XXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXX__, __XXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXX___, ___XXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXX____, ____XXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXX____, ____XXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXX_____, _____XXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXX_____, ______XX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XX_____X, __X___XX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XX____X_, _X___X_X, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, X___X_X_, X__X___X, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, X__X_X_X, _X_X___X, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, _X__X_X_, X_X_XX__, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, ___X_X_X, _X_X___X, _XXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, __X__X_X, X_X_X_X_, _XXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXX_, __X_XX_X, _X_X_X__, _XXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXX_, _X_X__X_, _X_X_X_X, _XXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXX_, X__X_XX_, X_X_X_X_, __XXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXX__, _X_X_X__, __XX_X__, X_XXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXX__, X_XX_X_X, X_X_XX_X, __XXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXX__, X_X_X_X_, X__X_X_X, __XXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXX_X_, _X_X_X_X, X_X_X_X_, X__XXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXX__, X_XX__X_, XX_X_X_X, _X_XXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXX__X, _X_X_XXX, X_X_XX_X, ___XXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXX_X_, _XX_XX_X, XXX__X_X, _X_XXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXX__X, _X__X_XX, _XX_X_X_, X__XXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXX__X, _X_X_XXX, XX_X_XX_, X__XXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXX_X_, XX_X_XXX, _XX__X_X, _X_XXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXX__X, _X__X_XX, XXXXX_X_, X__XXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXX__X_X, _X_XXX_X, XX_X__XX, _X_X_XXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, ______X_, XX_X_XXX, _XXX_XX_, X_______, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXX__, _____X_X, _X_XX_XX, XXX_X_X_, X_X_____, __XXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXX____, ___X__X_, XX_X_XX_, X_X_X_XX, _X__X___, ____XXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXX____, _X_X_X_X, _X___X_X, XXX___X_, X_X_X_X_, ____XXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXX___X, X_XXX_X_, X_X_X_X_, X____X_X, __X_XXXX, _X___XXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXX_XX_, XXXXX_X_, XX____X_, X_X_X_XX, _X_X_XX_, XX_XXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXX_X_X, X_XXX_X_, X_X___X_, X_____X_, X_XXXXXX, X_X_XXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXX_X_X, XX_XXX__, XX_X____, _X___X_X, __XX_XX_, XX__XXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXX_XXX, _XXX_X_X, X_X____X, ____X_XX, _X_XXXXX, X_X_XXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXX_X_X, XXXXXX_X, _XX_____, _____XX_, X_XXXX_X, _XX_XXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXX_X_, XX_X_X_X, _X_X____, ___X_X_X, _X_X_XXX, XX_XXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXX_XX, _XXXXX_X, X_XX_X__, _____XX_, X_XXXX_X, _X_XXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXX_X_, XX_X_X_X, _XX_____, _X__XX_X, _XX_XXXX, X_X_XXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXX__X_X, XXXXXXX_, X_XX_X__, ___X_XX_, X_XXX_X_, X_X__XXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXX_X_X_, XX_XXX_X, _X_X____, ___X_XX_, X_XXXXXX, XXX__XXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, X____XXX, XXXXXXXX, _XXX_X__, __X_XX_X, _XX_XXXX, _X__X__X, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, __X_X_X_, X_XX_XX_, X_X_X___, ____X_X_, XXXXX_XX, XX_X__X_, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXX_, X__X_X_X, XXXXXX_X, _X_XX_X_, __XXXXX_, X_XXXXX_, XX_X_X__, _XXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXX___, _X_X__XX, X_XXXXXX, _XXXX__X, ___X_X_X, _XXXXXXX, _X_X_X_X, _X_XXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXX_X_, X_X_X_X_, XXXX_XX_, XX__X_X_, _XXXX_X_, XXX_XX_X, XX__X_X_, __X_XXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXX___X, _X_X_X_X, _XXXXXXX, __XXXXX_, X_X_X_X_, XXXXXXXX, _X_X_X_X, _X__XXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XX__X_X_, X_X_X_XX, XXXXXXX_, X_X_X_X_, _XXX_X_X, _XXXXX_X, X_X_X_X_, X_X_X_XX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, X__X_X_X, _X_X_X_X, _X_XX_XX, X_X_XXX_, XXX_X_X_, XX_XXXXX, X_X_X_X_, X_X____X, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, _X__X_X_, X_X_X__X, XXXXXXXX, _X_XX_XX, XX_XX_X_, XXXXX_XX, X_X_X_X_, X_X_X_X_, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXX_, __X_X_XX, _X_XX_XX, X_XXXXXX, _X_X_X_X, X_XX_X_X, _XXXXXX_, X_X_X_XX, _X_X_X__, X_XXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXX__, X_X_X_X_, X_X_X_X_, XXXXX_XX, XX__XXX_, XXX_X_XX, XXXXX_XX, X_X_X_X_, XX_X_X_X, __XXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXX__X, _X_X_X_X, X_X_X_X_, X_XXXXXX, _X_XX_XX, XX_X_XX_, XXXXXXXX, _X_X_X_X, _X_X_X_X, _X__XXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXX__X_, X_X_X_X_, X_XX_X_X, XXXXXXXX, XX_X_X_X, X_XX_X_X, XXXXXX_X, X_X_XX_X, X_X_X_X_, X_X_XXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XX__X_XX, _X_X_X_X, X_X_X_X_, X_XXX_XX, X_X_X_XX, _XX_X_XX, X_XXXXXX, _X_X_X_X, _X_X_X_X, _X_X__XX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, X__X__X_, XX_XX_X_, XXX_XX_X, XXX_XXXX, _X__XX_X, X_X_X__X, XXX_X_XX, _X_XX_XX, _XX_XX_X, _X_X__XX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, __X_XX_X, _X_X_XXX, _X_XX_X_, X_XXXX_X, XXX_X_XX, _XX_XXXX, _XXXXX_X, _X_X_XX_, XX_XX_XX, _X_X_X__, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXX_, _X_X__X_, X_XXX_X_, XXX_X_X_, _XX_XXXX, XX_X_X_X, X_X_X_XX, XXX_XXXX, _X_XX_XX, X_XX_X_X, _XX_X_X_, _XXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXX__, X_X_XXX_, XX_X_XXX, _X_X_XX_, XXXXXXXX, _XXX_X_X, _X_X_X_X, XXXXX_X_, X_XX_X_X, _XX_X_XX, _X_X_X_X, __XXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXX_X_X, _X_X_X_X, X_XXXX_X, XXXXXX_X, X_XX_XXX, XXX_X_XX, X_X_XXXX, XXXXXX_X, _XX_XXXX, X_XXXX_X, _XX_X_X_, X___XXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXX__X_, X_X_XX_X, _X__X_X_, XX_X_X_X, _XXXXXXX, X_XX_X_X, _X_X_XX_, XXX_XXXX, _X_XX_X_, XXX_X_XX, X__XX_X_, X_X_XXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XX__X_X_, X_XX_X_X, XXXXXXXX, X_XX_X_X, _X_XXXXX, XXX_X_X_, XX__XXXX, XXXXX_X_, X_XX_XXX, X_XXXX_X, _XXX_X_X, X_X__X_X, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, X__X_X_X, _XX_X_X_, X_X_X_X_, XXX_XXX_, XXXXXXXX, XX_XX_X_, X_XXX_XX, XXXXXX_X, _XX_X_X_, XXX_X_XX, X_X_X_X_, X_X_X_XX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, __X_X_XX, _X__XXXX, XXXXXXXX, X_XX_X_X, _X_XX_XX, XXXXX_XX, _X_X_XXX, XX_XX_XX, _X_X_XXX, X_XXXXXX, _X_XX_XX, _XX_X___, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXX_, _X_X_XX_, X_XXXX_X, _X_X_XX_, XX_X_XX_, XXXXXXXX, _XX_X_X_, X_X_XXX_, XXXXXXX_, X_XXXX_X, XXXX_X_X, XXXX_XX_, X_X_X_X_, _XXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXX_X, _X_XX_X_, XXX_X_XX, XXXXXXXX, XXXXXX_X, _X_XX_XX, XX_X_X_X, _X_XX_XX, XXX_XX_X, _XXX_XXX, _X_XXXXX, X_X_XX_X, _X_XX_X_, X_XXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXX__X, _X_X_X_X, _X_XXXX_, XX_XX_X_, XXX_X_XX, _XX_XXXX, XXX_XX_X, _XXX_XXX, XXXXX_X_, X_X_X_XX, XXXXX_X_, XXXXX_XX, XX_X_XX_, _X_XXXXX, XXXXXXXX,
  XXXXXXXX, XXXX__X_, X_XX_XXX, XXXX_X_X, XXXXXXXX, X_XXXXX_, X_XXXXXX, XXXXXX_X, __X_XX_X, XXXXX_XX, _XXXXXX_, XXX_XXXX, X_X_XXX_, X_XX_X_X, XX__XXXX, XXXXXXXX,
  XXXXXXXX, XX_____X, _XX_XX_X, _X_XXXXX, _XX_XXXX, XXX_XX_X, _XX_XXX_, XXXX_X_X, _XXXXXXX, _XXXX_X_, X_X_X_XX, X_XXX_XX, _XXXX_XX, XX_X_XX_, X_____XX, XXXXXXXX,
  XXXXXXXX, XX_____X, XX_X_XXX, XXXX_XX_, XXXXXX_X, _XXXX_XX, _X_XXXXX, X_XXXXX_, XX_X_XXX, XXX_XXX_, XXXXXXXX, XXX_XXXX, XXX_XXXX, _XXXXX_X, X_____XX, XXXXXXXX,
  XXXXXXXX, ________, X_XXXX_X, XX_XXXXX, XX_X_XXX, XXX_XXX_, _XXXXXXX, XXX_XX_X, __XXXXXX, XXXXXX_X, _X_X_XX_, X_XXXXX_, XXXXX_XX, X_X_X_XX, ________, XXXXXXXX,
  XXXXXXX_, ________, X_X_XXXX, _XXX_XXX, _XXXXXXX, _XXXX_X_, XX_XXXXX, XXXXXXX_, XXX_XXXX, XXXXX_XX, _XXXXXXX, XXXXX_XX, X_XXXX_X, XXXXXXX_, ________, _XXXXXXX,
  XXXXXX__, ________, _XXXX_XX, XX_XXXXX, XXXX_XXX, XXX_XXX_, X_XXX_XX, XXX_X_X_, X_XXXX_X, XXX_XXX_, X_X_X_XX, XXX_XXX_, XXX_XXX_, XXX_X_XX, ________, __XXXXXX,
  XXXXX___, ________, XXX_XXX_, XXXXXX_X, _XXXXX_X, XXXXXX_X, _X_XXXXX, X_XXXXX_, XX_XXXXX, X_XXXX_X, _XXXXXX_, XXXXXXXX, XXXXX_XX, _XXXXXX_, X_______, ___XXXXX,
  XXX_____, _______X, _XXXX_XX, _X_XXXXX, XXX_XXXX, X_XX_XXX, __XXXXXX, XXXXX_XX, _XXX_XXX, XXXXX_XX, _XX_XXXX, X_XX_XX_, X_XXXXXX, XX_XXXXX, X_______, _____XXX,
  XXX_____, ______XX, XXX_XXXX, XXXXXX_X, _XXXXXXX, XXXXX_X_, XXX_XXXX, XXX_XXX_, X_XXXX_X, XXXX_XX_, X_XXX_XX, XXXXXXXX, XX_XXXXX, _XX_X_X_, XX_X____, _____XXX,
  X_______, ____XXX_, XX_XXX_X, XXXXX_XX, XXXX_XX_, XXX_XXX_, X_XXXXXX, XXXXXX_X, _XX_XXXX, _XXXXXX_, XXX_XXXX, XXX_XXX_, X_XXX_XX, XXXXXXXX, XXX_____, _______X,
  X_______, ___X_X_X, XXXXXXXX, XX_X_XX_, XXXXXXXX, XXXXXX_X, _X_X_XXX, _XX_X_X_, _X_XX_XX, XX_XX_X_, X_XXXXX_, XXXXXXXX, XXX_XXXX, X_XXXX_X, X_X_X___, _______X,
  ________, __X_XXXX, X_XX_XXX, XXXXXXXX, X_XXXXXX, XXXX_XXX, _X_XXX_X, XX_X_X_X, _X_X_XX_, X_XX_X_X, _XX_XXXX, XXXXXX_X, X_XX_XXX, XXXX_XXX, XX_XXX__, ________,
  X_______, X_XXX_X_, XXXXXXXX, _X_X_X_X, XXX_XXXX, XXXXXXX_, X__X_XXX, _XXXX_X_, _X_XX_XX, XXX_XX_X, _XXXXXXX, XX_X_XXX, XXXXXX_X, XXXXXXXX, _XXXX_X_, _______X,
  ________, _XX_XXXX, XXXXXXX_, XXXXXXXX, _XXXX_X_, X_X_XX_X, _XXXX_XX, XX_X_X_X, _X_X_XXX, _XXXX_X_, X_X_X_X_, XXXXXXXX, _XX_X_XX, _XXXXX_X, XX_X_XX_, X_______,
  X_______, XX_XX_XX, X_XXX_XX, XX_XXXXX, XXX_XXXX, XXXXX_XX, _X_X_XXX, _XXXX_X_, X_XXX_XX, XX_X_XX_, XXXXXXXX, XX_XX_XX, XXXXXXXX, X_XXXXXX, XXXXXX_X, _______X,
  X_______, X_X_XX_X, XXX_XXXX, _XXXX_XX, XXXXXXXX, XXX_XXX_, XX_XX_XX, XX_XX_X_, X_X_XXX_, XXX_XX_X, _XX_XXXX, XXXXXXXX, XXXXXX_X, XXX_XXXX, _XX_X_XX, _______X,
  XX______, _XXXXXXX, XXXXXX_X, XX_XXXXX, _XXX_XXX, XX_X_XX_, X_XX_X_X, XXXX_XX_, X_XX_XXX, XX_XX_X_, X_XXXXX_, XXXXX_XX, X_XX_XXX, _XXX_XXX, XXXXXXX_, ______XX,
  XXX_____, __XXXXXX, XXX_XXXX, XXXXXXXX, XXXXXX_X, XXXXXXX_, X_X_XXXX, X_XXX_X_, X_X_XXX_, XX_XX_X_, XXXXXXXX, XXXXXXXX, XXXXXXXX, XX_XXXXX, XXXX_XX_, ______XX,
  XXX_____, _XX_XXXX, X_XXXX_X, X_XXX_XX, XXXXXXXX, XXX_X_XX, _XXXX_XX, XXX_XX_X, _XXXXXXX, XXX_XXX_, X_XX_XXX, X_XXXXXX, XXXXXX_X, XXXXXX_X, _XXXXX__, _____XXX,
  XXXX____, __XXX_X_, XXXXXXXX, XXXXXXXX, XX_XXXXX, XX_XXXX_, X_X_XX_X, XXXXX_XX, _X_X_XX_, XX_XXX_X, _XXXXXXX, XXXXXXXX, XXX_XXXX, _XXX_XXX, XX_XXXX_, _____XXX,
  XXXX____, __XXXXXX, XX_XXXXX, XXXXXXXX, XXXXXXXX, XXXX_X_X, __XXX_XX, _XX_XXX_, _XXXXXXX, X_XXX__X, _X_XXXXX, XXXX_XXX, XXXXXXXX, XXXXXXXX, _XXXX___, ____XXXX,
  XXXXX___, ___X_X_X, XXXX_XXX, XX_XXXXX, XXXXXXXX, XXX_XXXX, _X_X_XXX, XXXXX_X_, XX_XX_XX, XXX_X_X_, XXXXXX_X, XXXXXX_X, XXXXXXXX, XXXXXXXX, XXX_XX__, ___XXXXX,
  XXXXX___, ___XXXXX, _XXXXXX_, XXXXXXX_, XXXX_XXX, _XXXX_X_, _X_XXX_X, _X_XX_X_, X_X_XXX_, XX_XX_X_, _X_XXXXX, XXXXXXXX, XXXXXX_X, XX_XXX_X, X_XXX___, ___XXXXX,
  XXXXXX__, ___X_X_X, XXXXXXXX, XXXXXXXX, XXXXXXXX, XX_X_XXX, _X_XX_XX, XXX_X_X_, X_XX_XX_, X_XXX_X_, XXXX_XXX, XXXXXXXX, XX_XXXXX, XXXXXXXX, XXX_X___, __XXXXXX,
  XXXXXX__, ____XXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXX_, _X_X_XXX, _X_XX_X_, X_X_X_XX, XXX_X_X_, _X_XXXXX, _XXXXXXX, XXXXXXXX, XXXXXXXX, _XXXX___, __XXXXXX,
  XXXXXXX_, ____X_XX, _X_X_X_X, X__X_XX_, XX_X_XX_, X_XXX_X_, X__XXX_X, XXXX_X_X, _X_XXXX_, XX_XX_X_, _XXXXX_X, XX_X__XX, _XXX_X_X, _XX_X_XX, XX_X____, _XXXXXXX,
  XXXXXXX_, ____XXXX, _X_X_X_X, _XXX_X_X, X_XXXX_X, XX_XXXX_, __X_X_X_, XXX_XX_X, _XX_X_XX, X_XXX__X, _X_XX_X_, X_XXXXX_, X_X_XX_X, _X_X_X_X, _XXX____, _XXXXXXX,
  XXXXXXXX, _____XX_, X__X_XX_, X_X_XXX_, X_X_X_XX, _X_XX_X_, X__XXXXX, X_XXX_XX, _X_XXXX_, XXX_X_X_, _XXXX_XX, XX_X_X_X, XX_XX_XX, _X_X_X__, XXX_____, XXXXXXXX,
  XXXXXXXX, X____XX_, X_X__X__, XX_X_X_X, XXXXXX_X, XXX_XXX_, __X_X_X_, XXX_X_X_, X_XX_XXX, XX_XX__X, _X_XX_XX, _XXXXXX_, X_XX_X_X, _X__X__X, _XX____X, XXXXXXXX,
  XXXXXXXX, X_X__X_X, _X_XX_XX, X__XX_X_, X_X_X_XX, _X_XX_XX, _X_XXXXX, XXXXXX_X, _X_XXXXX, _XXXX_X_, _XXX_XX_, XX_X_X_X, X_X_X_X_, X_XX_XX_, _XX____X, XXXXXXXX,
  XXXXXXXX, XX____X_, ________, __X___X_, _X_X_X_X, _X_X_X__, ___XX_XX, X_XXX_XX, _XXXX_XX, XX_XX___, X_X_X_X_, X_X_X_X_, _X__X___, X_______, X_____XX, XXXXXXXX,
  XXXXXXXX, XXXX_X_X, _X_X_X_X, _X_X_X_X, _X_X_X_X, _X_X_X_X, __X_XX_X, XXX_XXX_, _XX_XXX_, XX_XX_X_, ___X_X_X, _X_X_X_X, _X_X_X_X, _X_X_X_X, _X_XXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXX_, ___XX_XX, XXXXX_X_, XX_XXXXX, XXXXX___, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXX_, X__XXXXX, X_XXXX_X, _XXXXX_X, XX_XX___, _XXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, ___X_X__, _X_____X, ______X_, __XXX___, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, ___XXX_X, ____X___, X___X___, X_XXX___, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, X__XXX__, _X_____X, ______X_, __X_XX__, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, ___XXX_X, __X_____, _X__X__X, _XXXX___, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, X__X_X__, ____X__X, X_______, __XXX__X, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XX__XX_X, _X_____X, X___X_X_, X_X___XX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXX_____, ____X_XX, XXX_____, ____X_XX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX
};

GUI_CONST_STORAGE GUI_BITMAP bmSpaceShip = {
  128,              // xSize
  113,              // ySize
  16,               // BytesPerLine
  1,                // BitsPerPixel
  _acSpaceShip,     // Pointer to picture data (indices)
  &_PalSpaceShip,   // Pointer to palette
  NULL
};

/*************************** End of file ****************************/
