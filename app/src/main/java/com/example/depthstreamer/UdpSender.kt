package com.example.depthstreamer

import android.util.Log
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.nio.ByteBuffer
import java.nio.ByteOrder

class UdpSender {
    private var socket: DatagramSocket? = null

    init {
        try {
            socket = DatagramSocket()
        } catch (e: Exception) {
            Log.e("UdpSender", "Failed to create socket", e)
        }
    }

    suspend fun sendDepthData(
        ipAddress: String,
        port: Int,
        width: Int,
        height: Int,
        rowStride: Int,
        depthBuffer: ByteBuffer
    ): String? = withContext(Dispatchers.IO) {
        if (socket == null) return@withContext "Socket is null"

        try {
            val address = InetAddress.getByName(ipAddress)
            
            val maxPayloadSize = 60000
            val bytesPerRow = width * 2
            val maxRowsPerPacket = maxPayloadSize / bytesPerRow
            
            for (yOffset in 0 until height step maxRowsPerPacket) {
                val yCount = minOf(maxRowsPerPacket, height - yOffset)
                val dataBytes = yCount * bytesPerRow
                val packetData = ByteArray(16 + dataBytes)
                
                val wrapper = ByteBuffer.wrap(packetData)
                wrapper.order(ByteOrder.LITTLE_ENDIAN)
                wrapper.putInt(width)
                wrapper.putInt(height)
                wrapper.putInt(yOffset)
                wrapper.putInt(yCount)
                
                for (y in 0 until yCount) {
                    depthBuffer.position((yOffset + y) * rowStride)
                    depthBuffer.get(packetData, 16 + y * bytesPerRow, bytesPerRow)
                }
                
                val packet = DatagramPacket(packetData, packetData.size, address, port)
                socket?.send(packet)
            }
            null // success, no error
        } catch (e: Exception) {
            Log.e("UdpSender", "Failed to send UDP packet", e)
            e.message // return error string
        }
    }

    fun close() {
        socket?.close()
    }
}
