package com.example.depthstreamer

import android.Manifest
import android.annotation.SuppressLint
import android.content.Context
import android.content.pm.PackageManager
import android.graphics.ImageFormat
import android.hardware.camera2.*
import android.media.Image
import android.media.ImageReader
import android.os.Bundle
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.unit.dp
import androidx.core.content.ContextCompat
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import java.nio.ByteBuffer

class MainActivity : ComponentActivity() {

    private val udpSender = UdpSender()
    
    // State for UI
    private var depthResolution by mutableStateOf("N/A")
    private var centerDistanceMm by mutableStateOf(0)
    private var centerConfidence by mutableStateOf(0)
    private var fps by mutableStateOf(0)
    private var ipAddress by mutableStateOf("192.168.100.28")
    private var port by mutableStateOf("5005")
    private var isStreaming by mutableStateOf(false)
    private var errorMessage by mutableStateOf("")

    // FPS tracking
    private var frameCount = 0
    private var lastFpsTime = System.currentTimeMillis()

    // Camera2 variables
    private var cameraDevice: CameraDevice? = null
    private var captureSession: CameraCaptureSession? = null
    private var imageReader: ImageReader? = null
    private var backgroundThread: HandlerThread? = null
    private var backgroundHandler: Handler? = null

    private val requestPermissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestPermission()
    ) { isGranted: Boolean ->
        if (isGranted) {
            startCameraThread()
            openCamera()
        } else {
            errorMessage = "Camera permission denied"
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        
        setContent {
            MaterialTheme {
                Surface(
                    modifier = Modifier.fillMaxSize(),
                    color = MaterialTheme.colorScheme.background
                ) {
                    DepthStreamerUI(
                        depthResolution = depthResolution,
                        centerDistanceMm = centerDistanceMm,
                        centerConfidence = centerConfidence,
                        fps = fps,
                        ipAddress = ipAddress,
                        onIpAddressChange = { ipAddress = it },
                        port = port,
                        onPortChange = { port = it },
                        isStreaming = isStreaming,
                        errorMessage = errorMessage,
                        onStreamingToggle = { isStreaming = it }
                    )
                }
            }
        }

        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) 
            == PackageManager.PERMISSION_GRANTED) {
            startCameraThread()
            openCamera()
        } else {
            requestPermissionLauncher.launch(Manifest.permission.CAMERA)
        }
    }

    private fun startCameraThread() {
        backgroundThread = HandlerThread("CameraBackground").also { it.start() }
        backgroundHandler = Handler(backgroundThread!!.looper)
    }

    private fun stopCameraThread() {
        backgroundThread?.quitSafely()
        try {
            backgroundThread?.join()
            backgroundThread = null
            backgroundHandler = null
        } catch (e: InterruptedException) {
            Log.e("DepthStreamer", "Interrupted while stopping background thread", e)
        }
    }

    @SuppressLint("MissingPermission")
    private fun openCamera() {
        val manager = getSystemService(Context.CAMERA_SERVICE) as CameraManager
        try {
            var depthCameraId: String? = null
            
            // Find rear depth camera
            for (cameraId in manager.cameraIdList) {
                val chars = manager.getCameraCharacteristics(cameraId)
                val facing = chars.get(CameraCharacteristics.LENS_FACING)
                val capabilities = chars.get(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES)
                
                if (facing == CameraCharacteristics.LENS_FACING_BACK &&
                    capabilities?.contains(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES_DEPTH_OUTPUT) == true) {
                    depthCameraId = cameraId
                    break
                }
            }

            if (depthCameraId == null) {
                errorMessage = "No depth camera found on this device."
                return
            }

            // Create ImageReader for 240x180 DEPTH16
            val width = 240
            val height = 180
            imageReader = ImageReader.newInstance(width, height, ImageFormat.DEPTH16, 2)
            imageReader?.setOnImageAvailableListener({ reader ->
                val image = reader.acquireLatestImage()
                if (image != null) {
                    processDepthImage(image)
                    image.close()
                    
                    // Update FPS
                    frameCount++
                    val currentTime = System.currentTimeMillis()
                    if (currentTime - lastFpsTime >= 1000) {
                        fps = frameCount
                        frameCount = 0
                        lastFpsTime = currentTime
                    }
                }
            }, backgroundHandler)

            manager.openCamera(depthCameraId, object : CameraDevice.StateCallback() {
                override fun onOpened(camera: CameraDevice) {
                    cameraDevice = camera
                    startCaptureSession()
                }

                override fun onDisconnected(camera: CameraDevice) {
                    camera.close()
                    cameraDevice = null
                }

                override fun onError(camera: CameraDevice, error: Int) {
                    camera.close()
                    cameraDevice = null
                    errorMessage = "Camera open error: $error"
                }
            }, backgroundHandler)

        } catch (e: CameraAccessException) {
            errorMessage = "Camera access exception: ${e.message}"
        } catch (e: Exception) {
            errorMessage = "Failed to open camera: ${e.message}"
        }
    }

    private fun startCaptureSession() {
        val camera = cameraDevice ?: return
        val surface = imageReader?.surface ?: return

        try {
            val captureRequestBuilder = camera.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW)
            captureRequestBuilder.addTarget(surface)

            camera.createCaptureSession(listOf(surface), object : CameraCaptureSession.StateCallback() {
                override fun onConfigured(session: CameraCaptureSession) {
                    if (cameraDevice == null) return
                    captureSession = session
                    try {
                        captureRequestBuilder.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_PICTURE)
                        session.setRepeatingRequest(captureRequestBuilder.build(), null, backgroundHandler)
                    } catch (e: CameraAccessException) {
                        errorMessage = "Failed to start camera preview: ${e.message}"
                    }
                }

                override fun onConfigureFailed(session: CameraCaptureSession) {
                    errorMessage = "Camera capture session configuration failed"
                }
            }, backgroundHandler)
        } catch (e: CameraAccessException) {
            errorMessage = "Camera capture session error: ${e.message}"
        }
    }

    private fun processDepthImage(image: Image) {
        val width = image.width
        val height = image.height
        depthResolution = "${width}x${height}"

        val planes = image.planes
        if (planes.isNotEmpty()) {
            val buffer = planes[0].buffer
            val rowStride = planes[0].rowStride
            val pixelStride = planes[0].pixelStride
            
            // Calculate center pixel distance and confidence
            val centerX = width / 2
            val centerY = height / 2
            val centerIndex = (centerY * rowStride) + (centerX * pixelStride)
            
            if (centerIndex < buffer.capacity()) {
                val depthShort = buffer.getShort(centerIndex)
                val depthInt = depthShort.toInt()
                // In Camera2 DEPTH16 format:
                // Least significant 13 bits are distance in mm
                // Most significant 3 bits are confidence (0 = 0%, 7 = 100%)
                centerDistanceMm = depthInt and 0x1FFF
                centerConfidence = (depthInt shr 13) and 0x7
            }

            if (isStreaming) {
                val portInt = port.toIntOrNull() ?: 5005
                val depthDataSize = buffer.remaining()
                val copiedData = ByteArray(depthDataSize)
                buffer.get(copiedData)
                val copiedBuffer = ByteBuffer.wrap(copiedData)
                
                CoroutineScope(Dispatchers.IO).launch {
                    val udpError = udpSender.sendDepthData(ipAddress, portInt, width, height, rowStride, copiedBuffer)
                    if (udpError != null) {
                        errorMessage = "UDP Error: $udpError"
                    }
                }
            }
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        captureSession?.close()
        cameraDevice?.close()
        imageReader?.close()
        stopCameraThread()
        udpSender.close()
    }
}

@Composable
fun DepthStreamerUI(
    depthResolution: String,
    centerDistanceMm: Int,
    centerConfidence: Int,
    fps: Int,
    ipAddress: String,
    onIpAddressChange: (String) -> Unit,
    port: String,
    onPortChange: (String) -> Unit,
    isStreaming: Boolean,
    errorMessage: String,
    onStreamingToggle: (Boolean) -> Unit
) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(16.dp)
    ) {
        Text(text = "Raw ToF Depth Streamer", style = MaterialTheme.typography.headlineMedium)
        
        if (errorMessage.isNotEmpty()) {
            Text(text = "Error: $errorMessage", color = MaterialTheme.colorScheme.error)
        }

        Card(modifier = Modifier.fillMaxWidth()) {
            Column(modifier = Modifier.padding(16.dp)) {
                Text("Stats", style = MaterialTheme.typography.titleMedium)
                Spacer(modifier = Modifier.height(8.dp))
                Text("Resolution: $depthResolution")
                Text("Center Distance: $centerDistanceMm mm")
                Text("Center Confidence: $centerConfidence/7")
                Text("FPS: $fps")
            }
        }

        OutlinedTextField(
            value = ipAddress,
            onValueChange = onIpAddressChange,
            label = { Text("PC IP Address") },
            modifier = Modifier.fillMaxWidth()
        )

        OutlinedTextField(
            value = port,
            onValueChange = onPortChange,
            label = { Text("UDP Port") },
            keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
            modifier = Modifier.fillMaxWidth()
        )

        Button(
            onClick = { onStreamingToggle(!isStreaming) },
            modifier = Modifier.fillMaxWidth()
        ) {
            Text(if (isStreaming) "Stop Streaming" else "Start Streaming")
        }
    }
}
