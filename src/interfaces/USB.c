#include "USB.h"
#include "CDFormat.h"
#include "3DO.h"
#include "CDROM.h"
#include "MSC.h"
#include "pico/stdio.h"

uint8_t usb_state = 0;

static int deviceCnt = 0;

bool usb_cmd_on_going = false;

static scsi_inquiry_resp_t inquiry_resp;
static bool inquiry_complete_cb(uint8_t dev_addr, msc_cbw_t const* cbw, msc_csw_t const* csw);
static scsi_sense_fixed_resp_t cd_sense_response[CFG_TUH_DEVICE_MAX];
static bool cd_sense_complete_cb(uint8_t dev_addr, msc_cbw_t const* cbw, msc_csw_t const* csw);

static device_s devices[CFG_TUH_DEVICE_MAX] = {0};

device_s* getDevice(uint8_t dev_addr) {
  for (int i = 0; i<CFG_TUH_DEVICE_MAX; i++) {
    if (devices[i].dev_addr == dev_addr)
      return &devices[i];
  }
  for (int i = 0; i<CFG_TUH_DEVICE_MAX; i++) {
    if (devices[i].dev_addr == 0xFF) {
      devices[i].dev_addr = dev_addr;
      return &devices[i];
      break;
    }
  }
  return NULL;
}

device_s* getDeviceIndex(uint8_t idx) {
  if(idx >= CFG_TUH_DEVICE_MAX) return NULL;
  else return &devices[idx];
}

void USB_Host_init() {
    for (int i = 0; i<CFG_TUH_DEVICE_MAX; i++) {
      devices[i].dev_addr = 0xFF;
      devices[i].useable = false;
      devices[i].isFatFs = false;
      devices[i].canBeLoaded = false;
      devices[i].canBeEjected = false;
      devices[i].cdProbePending = false;
      devices[i].cdTrayStatePending = false;
      devices[i].cdMediaReady = false;
      devices[i].state = DETACHED;
      devices[i].type = UNKNOWN_TYPE;
      devices[i].tray_open = false;
    }
    currentImage.curDir = NULL;
    currentImage.curPath = NULL;
    currentImage.dev_addr = 0xFF;
    currentImage.lun = 0x0;
    usb_cmd_on_going = false;
    deviceCnt = 0;
#ifdef USE_TRACE
    stdio_init_all();
#endif
    tusb_init();
}

static bool check_eject(device_s *dev);

static bool Default_Host_loop(device_s *dev) {
#if CFG_TUH_MSC
  if (!usb_cmd_on_going) {
    switch(dev->state) {
      case EJECTING:
        check_eject(dev);
      case MOUNTED:
        return true;
        break;
      default:
        return false;
    }
  }
#endif
  return false;
}

void USB_Host_loop()
{
  // tinyusb host task
  bool ret = true;
  tuh_task();
  for (int i =0; i<CFG_TUH_DEVICE_MAX; i++) {
    device_s *dev = getDeviceIndex(i);
    if (dev->dev_addr == 0xFF) continue;
    switch (dev->type) {
      case CD_TYPE:
        ret = CDROM_Host_loop(dev);
        break;
      case MSC_TYPE:
        ret = MSC_Host_loop(dev);
        break;
      default:
        ret = Default_Host_loop(dev);
      }
      if (!ret) {
        if(dev->state==INQUIRY) {
          dev->state = ATTACHED;
          tuh_msc_inquiry(dev->dev_addr, 0, &inquiry_resp, inquiry_complete_cb);
        }
      }
  }
}


#if CFG_TUH_MSC

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

volatile uint8_t requestLoad = 0xFF;
cd_s currentImage = {0};

static bool startClose = true;

volatile bool read_done;
volatile bool is_audio;
volatile bool has_subQ;

volatile bool usb_result;

uint32_t start_Block;
uint32_t nb_block_Block;
uint8_t *buffer_Block;
bool blockRequired = false;
bool subqRequired = false;
uint8_t *buffer_subq;

fileCmd_s fileCmdRequired;

static bool check_eject(device_s *dev) {
  //Execute right now
  LOG_SATA("Eject\n");
  if (CDROM_ExecuteEject(dev->dev_addr)) dev->state = INQUIRY;
  return true;
}

void USB_reset() {
  LOG_SATA("reset usb\n");
  startClose = false;
  for (int i=0; i<CFG_TUH_DEVICE_MAX; i++) {
    device_s *dev = getDeviceIndex(i);
      tuh_msc_umount_cb(dev->dev_addr);
  }
  tusb_reset();
}

static bool inquiry_complete_cb(uint8_t dev_addr, msc_cbw_t const* cbw, msc_csw_t const* csw)
{
  device_s *dev = getDevice(dev_addr);
  if (csw->status != MSC_CSW_STATUS_GOOD)
  {
    TU_LOG1("Inquiry failed %x\r\n", csw->status);
    dev->state = INQUIRY;
    return false;
  }
  //At least consider we are enumerated so that configuration can be read
  if (dev->state < ENUMERATED) dev->state = ENUMERATED;
  // Print out Vendor ID, Product ID and Rev
  TU_LOG1("%.8s %.16s rev %.4s Type 0x%x Lun %d\r\n", inquiry_resp.vendor_id, inquiry_resp.product_id, inquiry_resp.product_rev, inquiry_resp.peripheral_device_type, cbw->lun);

  dev->dev_addr = dev_addr;

  dev->lun = cbw->lun;

  if (inquiry_resp.peripheral_device_type == 0x5) {
    deviceCnt++;
    dev->type = CD_TYPE;
  }
  if (inquiry_resp.peripheral_device_type == 0x0) {
    deviceCnt++;
    dev->type = MSC_TYPE;
  }
  checkForMedia(dev_addr, dev->lun);

  return true;
}

int64_t alarm_callback(alarm_id_t id, void *user_data) {
    device_s *dev = (device_s *)user_data;
    checkForMedia(dev->dev_addr, dev->lun);
    // Can return a value here in us to fire in the future
    return 0;
}

void tryCheckForMedia(device_s *dev) {
  add_alarm_in_ms(500, alarm_callback, (void*)dev, false);
}

static bool cd_sense_complete_cb(uint8_t dev_addr, msc_cbw_t const* cbw, msc_csw_t const* csw) {
  (void)cbw;
  if (dev_addr == 0 || dev_addr > CFG_TUH_DEVICE_MAX) return false;

  device_s *dev = getDevice(dev_addr);
  if (dev == NULL || dev->type != CD_TYPE) return false;

  scsi_sense_fixed_resp_t const* sense = &cd_sense_response[dev_addr - 1];
  dev->tray_open = csw->status == MSC_CSW_STATUS_GOOD &&
      ((sense->response_code & 0x7F) == 0x70 || (sense->response_code & 0x7F) == 0x71) &&
      sense->sense_key == SCSI_SENSE_NOT_READY &&
      sense->add_sense_code == 0x3A && sense->add_sense_qualifier == 0x02;
  dev->cdTrayStatePending = false;
  mediaInterrupt();
  return true;
}

void tuh_msc_ready_cb(uint8_t dev_addr, bool ready) {
  if (dev_addr == 0 || dev_addr > CFG_TUH_DEVICE_MAX) return;
  device_s *dev=getDevice(dev_addr);
  if (dev->type == CD_TYPE) {
    CDROM_ready(dev_addr, ready);
    if (!ready) {
      dev->cdProbePending = false;
      dev->cdMediaReady = false;
      if (!dev->cdTrayStatePending) {
        dev->cdTrayStatePending = true;
        memset(&cd_sense_response[dev_addr - 1], 0, sizeof(cd_sense_response[dev_addr - 1]));
        if (!tuh_msc_request_sense(dev_addr, dev->lun, &cd_sense_response[dev_addr - 1],
                                   cd_sense_complete_cb)) {
          dev->cdTrayStatePending = false;
          dev->tray_open = false;
        }
      }
      mediaInterrupt();
    } else {
      dev->cdTrayStatePending = false;
      dev->tray_open = false;
      if (!dev->cdProbePending && !dev->cdMediaReady) {
        CDROM_Inquiry(dev_addr, dev->lun);
      }
    }
  }
  if (!ready) {
    tryCheckForMedia(dev);
    //Voir pour executer le check for media tous les 300 ms au plus rapide
  }
}

void tuh_msc_enumerate_cb (uint8_t dev_addr) {
  device_s *dev = getDevice(dev_addr);
  TU_LOG1("Usb device enumerated %x\n", dev_addr);
  dev->dev_addr = dev_addr;
  dev->state = INQUIRY;
}

bool read_complete_cb(uint8_t dev_addr, msc_cbw_t const* cbw, msc_csw_t const* csw) {
  if (csw->status != MSC_CSW_STATUS_GOOD) {
    set3doDriveError();
  }
  read_done = true;
  usb_cmd_on_going = false;
  return true;
}

bool block_is_ready() {
  return read_done;
}

bool cmd_is_ready(bool *success) {
  if (success != NULL) *success = usb_result;
  return fileCmdRequired == DONE;
}

bool write_is_ready(bool *success) {
  *success = usb_result;
  return fileCmdRequired == DONE;
}

bool USBDriveEject(uint8_t dev_addr, bool eject) {
  device_s *dev = getDevice(dev_addr);
  if (dev->type == CD_TYPE) {
    dev->state = EJECTING;
    return true;
  }
  if (dev->type == MSC_TYPE) {
    return true;
  }
  return false;
}

int getDeviceCount(void) {
  return deviceCnt;
}


bool readSubQChannel(uint8_t *buffer) {
  buffer_subq = buffer;
  read_done = false;
  subqRequired = true;
  return true;
}

bool isAudioBlock(uint32_t start) {
  bool res = false;
  for (int i = 0; i < currentImage.nb_track-1; i++) {
    if (currentImage.tracks[i].lba <= start) {
      res = currentImage.tracks[i].mode == CDDA;
    }
  }
  return res;
}

bool readBlock(uint32_t start, uint16_t nb_block, uint16_t block_size, uint8_t *buffer) {
  is_audio = false;
  has_subQ = false;
  read_done = false;
  start_Block = start;
  nb_block_Block = nb_block;
  buffer_Block = buffer;
  for (int i = 0; i < currentImage.nb_track-1; i++) {
    if (currentImage.tracks[i].lba <= start) {
      is_audio = currentImage.tracks[i].mode == CDDA;
      has_subQ = (block_size >= 2448);
    }
  }
  blockRequired = true;
  return true;
}
//------------- IMPLEMENTATION -------------//

void tuh_msc_enumerated_cb(uint8_t dev_addr)
{
  TU_LOG1("##### enumerated ######\n");
}

void tuh_msc_mount_cb(uint8_t dev_addr)
{
  bool ret = false;
  device_s *dev = getDevice(dev_addr);
  LOG_SATA("A USB MassStorage device is mounted %d\r\n", dev_addr);
  dev->state = MOUNTED;

  if (dev->type == CD_TYPE) {
    CDROM_Inquiry(dev_addr, dev->lun);
  }
  if (dev->type == MSC_TYPE) {
    MSC_Inquiry(dev_addr, dev->lun);
  }
}

void tuh_msc_umount_cb(uint8_t dev_addr)
{
  device_s *dev = getDevice(dev_addr);

  LOG_SATA("A USB MassStorage device is unmounted\r\n");
  if (requestLoad == dev_addr) requestLoad = 0xFF;

  //A voir si la currentImage utilise le dev_addr en question...
  //Pour le moment on va dire oui
  {
    set3doCDReady(dev_addr, false);
    set3doDriveMounted(dev_addr, false);
  }

  if (currentImage.dev_addr == dev_addr) {
    LOG_SATA("Unmount USB key used for the currentImage!\n");
    if (currentImage.curDir != NULL) free(currentImage.curDir);
    if (currentImage.curPath != NULL) free(currentImage.curPath);
    currentImage.curDir = NULL;
    currentImage.curPath = NULL;
    currentImage.dev_addr = 0xFF;
    currentImage.lun = 0x0;
  }
  dev->dev_addr = 0xFF;
  dev->useable = false;
  dev->isFatFs = false;
  dev->canBeLoaded = false;
  dev->canBeEjected = false;
  dev->cdProbePending = false;
  dev->cdTrayStatePending = false;
  dev->cdMediaReady = false;
  dev->state = DETACHED;
  dev->type = UNKNOWN_TYPE;
  deviceCnt--;
  mediaInterrupt();
}

#endif
