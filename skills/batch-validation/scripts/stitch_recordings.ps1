param(
    [Parameter(Mandatory = $true)]
    [string]$ExportDir,
    [Parameter(Mandatory = $true)]
    [string]$Manifest,
    [int]$Width = 1280,
    [int]$Height = 720,
    [int]$Crf = 25
)

$ErrorActionPreference = "Stop"
$doc = Get-Content -Raw -Path $Manifest | ConvertFrom-Json

foreach ($run in $doc.runs) {
    $key = $run.key
    $slug = if ($run.slug) { $run.slug } else { "$key-$($run.exec_id)" }
    $fragDir = Join-Path $ExportDir ("raw_frags\" + $key)
    $segs = Get-ChildItem -Path $fragDir -Filter "seg_*.mp4" | Sort-Object Name
    if ($segs.Count -eq 0) {
        throw "No raw segments found in $fragDir"
    }

    $outPath = Join-Path $ExportDir ($slug + ".mp4")
    $inputs = @()
    $filters = @()
    for ($i = 0; $i -lt $segs.Count; $i++) {
        $inputs += @("-i", $segs[$i].FullName)
        $filters += "[$i`:v]scale=${Width}:${Height}:flags=lanczos,setsar=1,fps=30[v$i]"
    }
    $concatIn = (0..($segs.Count - 1) | ForEach-Object { "[v$_]" }) -join ""
    $fc = ($filters -join ";") + ";${concatIn}concat=n=$($segs.Count):v=1:a=0[vout]"

    Write-Host "Stitching $key ($($segs.Count) segments) -> $outPath ..."
    & ffmpeg -y -loglevel error @inputs -filter_complex $fc -map "[vout]" `
        -c:v libx264 -preset medium -crf $Crf -pix_fmt yuv420p -movflags +faststart $outPath
    if ($LASTEXITCODE -ne 0) {
        throw "ffmpeg failed for $key"
    }
    $sizeMB = [math]::Round((Get-Item $outPath).Length / 1MB, 2)
    Write-Host "  -> $outPath ($sizeMB MB)"
    if ($sizeMB -ge 9.8) {
        Write-Warning "$outPath is $sizeMB MB (close to GitHub's 10 MB attachment limit); rerun with higher -Crf."
    }
}
