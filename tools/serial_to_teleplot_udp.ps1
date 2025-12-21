param(
  [string]$Port = 'COM3',
  [int]$Baud = 115200,
  [string]$UdpHost = '127.0.0.1',
  [int]$UdpPort = 47269,
  [switch]$Timestamp
)

# Forwards Teleplot-serial lines from the MCU:
#   >name:value\n
# To Teleplot UDP:
#   name:value|g
# or with timestamp (ms):
#   name:timestamp_ms:value|g

$sp = New-Object System.IO.Ports.SerialPort $Port,$Baud,'None',8,'One'
$sp.NewLine = "`n"
$sp.ReadTimeout = 250
$sp.DtrEnable = $true
$sp.RtsEnable = $true

$udp = New-Object System.Net.Sockets.UdpClient
$remote = New-Object System.Net.IPEndPoint ([System.Net.IPAddress]::Parse($UdpHost)), $UdpPort

function Send-Teleplot([string]$name, [string]$value) {
  if ($Timestamp) {
    $ms = [int64]([DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds())
    $msg = "$name`:$ms`:$value|g"
  } else {
    $msg = "$name`:$value|g"
  }
  $bytes = [System.Text.Encoding]::UTF8.GetBytes($msg)
  [void]$udp.Send($bytes, $bytes.Length, $remote)
}

try {
  $sp.Open()
  Write-Host "Serial open: $Port @ $Baud (RTS+DTR forced)"
  Write-Host "Forwarding to UDP: $UdpHost`:$UdpPort" 

  while ($true) {
    try {
      $line = $sp.ReadLine()
    } catch [System.TimeoutException] {
      continue
    }

    if (-not $line) { continue }
    $t = $line.Trim()

    # Teleplot serial format: >name:value
    if ($t.StartsWith('>')) {
      $payload = $t.Substring(1)
      $idx = $payload.IndexOf(':')
      if ($idx -gt 0 -and $idx -lt ($payload.Length - 1)) {
        $name = $payload.Substring(0, $idx).Trim()
        $value = $payload.Substring($idx + 1).Trim()
        if ($name.Length -gt 0 -and $value.Length -gt 0) {
          Send-Teleplot $name $value
        }
      }
    }
  }
} finally {
  try { if ($sp.IsOpen) { $sp.Close() } } catch {}
  try { $udp.Close() } catch {}
}
