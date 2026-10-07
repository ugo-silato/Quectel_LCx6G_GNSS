/*
 * ============================================================================
 *  LC76G_Params.cpp  -  Estimated positioning error, parameter management
 * ============================================================================
 *  Part of the Quectel_LCx6G_GNSS Arduino library. Author: Ugo Silato. License: MIT.
 *
 *  Commands used (Quectel GNSS Protocol Specification):
 *    PQTMCFGMSGRATE  Output rate of the PQTMEPE message (version 2)
 *    PQTMRESTOREPAR  Restore all parameters to the default values
 *    PQTMSAVEPAR     Save the configuration to flash
 *  The $PQTMEPE message itself is decoded in LC76G::handleSentence().
 * ============================================================================
 */

#include "LC76G.h"

LC76G_Result LC76G::setEpeOutput(uint8_t rate) {
  if (rate > 20) {
    return LC76G_PARAM_ERROR;
  }
  return setPqtmMessageRate("PQTMEPE", rate, 2);   // PQTMEPE is message version 2
}

uint32_t LC76G::epeAgeMs() const {
  if (_epeMs == 0) {
    return 0xFFFFFFFFUL;                       // Nothing received yet
  }
  return millis() - _epeMs;
}

LC76G_Result LC76G::restoreDefaults() {
  // Writing the defaults to flash can take a moment: allow 3 s
  LC76G_Result r = sendPqtm("PQTMRESTOREPAR", nullptr, 3000);
  if (r == LC76G_OK) {
    // The defaults take effect only after a restart
    if (!reboot()) {
      return LC76G_NO_REBOOT;                  // No power/reset pin: power-cycle manually
    }
    waitCommandReady(LC76G_READY_MS);          // Commands accepted again
  }
  return r;
}

LC76G_Result LC76G::saveParameters() {
  return sendPqtm("PQTMSAVEPAR", nullptr, 3000);
}
