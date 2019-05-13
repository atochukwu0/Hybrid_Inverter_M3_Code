//###########################################################################
// FILE:    F28M35x_MemCopy.c
// TITLE:    Memory Copy Utility
// ASSUMPTIONS:
// DESCRIPTION:
//          This function will copy the specified memory contents from
//          one location to another.
//          Uint16 *SourceAddr        Pointer to the first word to be moved
//                                    SourceAddr < SourceEndAddr
//          Uint16* SourceEndAddr     Pointer to the last word to be moved
//          Uint16* DestAddr          Pointer to the first destination word
//          No checks are made for invalid memory locations or that the
//          end address is > then the first start address.
//###########################################################################
// $TI Release: F28M35x Driver Library vAlpha1 $
// $Release Date: July 11, 2011 $
//###########################################################################

#include "utils/memcopy.h"

//*****************************************************************************
//! \addtogroup memcopy_api
//! @{
//*****************************************************************************


//*****************************************************************************
//! This function will copy the specified memory contents from
//! one location to another.
//!
//! \param SourceAddr is the pointer to the first word to be moved where
//! SourceAddr < SourceEndAddr
//! \param SourceEndAddr is the pointer to the last word to be moved
//! \param DestAddr is the pointer to the first destination word.
//!
//! This function will copy the specified memory contents from
//! one location to another. It can be used to copy contents of Flash
//! to Ram during runtime for faster execution. 
//!
//! No checks are made for invalid memory locations or that the
//! end address is > then the first start address.
//!
//! \return None.
//*****************************************************************************
void MemCopy(unsigned long *SourceAddr, unsigned long* SourceEndAddr, unsigned long* DestAddr)
{
    while(SourceAddr < SourceEndAddr)
    {
        *DestAddr++ = *SourceAddr++;
    }
    return;
}

//*****************************************************************************
// Close the Doxygen group.
//! @}
//*****************************************************************************
