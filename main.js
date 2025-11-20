// main.js
const { app, BrowserWindow, session, dialog } = require('electron');
const path = require('path');

let mainWindow;

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1200,
    height: 800,
    webPreferences: {
      preload: path.join(__dirname, 'preload.js'),
      contextIsolation: true,
      nodeIntegration: false
    }
  });

  mainWindow.loadFile(path.join(__dirname, 'index.html'));

  mainWindow.on('closed', () => {
    mainWindow = null;
  });
}

app.whenReady().then(() => {
  const ses = session.defaultSession;

  // 1) Allow serial devices
  ses.setDevicePermissionHandler((details) => {
    if (details.deviceType === 'serial') {
      return true; // allow all serial devices
    }
    return false;
  });

  // 2) Always say "yes" for serial permission checks
  ses.setPermissionCheckHandler((webContents, permission, requestingOrigin, details) => {
    if (permission === 'serial') {
      return true;
    }
    return false;
  });

  // 3) Show a COM-port chooser instead of auto-select
  ses.on('select-serial-port', (event, portList, webContents, callback) => {
    event.preventDefault();

    if (!portList || portList.length === 0) {
      dialog.showMessageBoxSync(mainWindow, {
        type: 'warning',
        title: 'No serial ports',
        message: 'No serial ports were found. Please connect your device and try again.'
      });
      callback(''); // cancel
      return;
    }

    // Build a list of button labels like "COM3 (USB-SERIAL...)" or fallback to deviceId
    const buttonLabels = portList.map((p, idx) => {
      const name = p.portName || p.displayName || p.deviceId || `Port ${idx + 1}`;
      return name;
    });

    // Add a cancel button at the end
    buttonLabels.push('Cancel');

    const choice = dialog.showMessageBoxSync(mainWindow, {
      type: 'question',
      buttons: buttonLabels,
      title: 'Select Serial Port',
      message: 'Select the serial port to use for HexConsole:',
      cancelId: buttonLabels.length - 1,
      defaultId: 0
    });

    // If user clicked "Cancel"
    if (choice === buttonLabels.length - 1) {
      callback(''); // cancel
      return;
    }

    // Map selected button index to the portId
    const selectedPort = portList[choice];
    if (selectedPort) {
      console.log('User selected port:', selectedPort.portName || selectedPort.deviceId);
      callback(selectedPort.portId);
    } else {
      callback(''); // safety
    }
  });

  // Optional tweaks (can help with some setups)
  app.commandLine.appendSwitch('disable-serial-blocklist');
  app.commandLine.appendSwitch('enable-experimental-web-platform-features');

  createWindow();

  app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) {
      createWindow();
    }
  });
});

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') {
    app.quit();
  }
});
