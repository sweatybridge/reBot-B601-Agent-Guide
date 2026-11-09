# One-shot repeat of the no-measurement Pinocchio trajectory.
#
#   pwsh -File examples/host/deploy/repeat_pino.ps1            # plan + save the JSON
#   pwsh -File examples/host/deploy/repeat_pino.ps1 -Replay    # plan, save, and play it on the arm
#
# Needs: Docker Desktop with the rebot-pino image (built from
# examples/host/Dockerfile.pinocchio), ssh access to the Pi, and the Rust edge
# binary at $PiDir/rebot-edge-rust. The planner runs in the container because
# Pinocchio has no Windows wheels; the pose dump and the replay go over ssh.
param(
    [string]$Pi = "qh812@raspberrypi",
    [string]$PiDir = "~/gravity-test",
    [string]$Image = "rebot-pino",
    [string]$Out = "examples/trajectories/seq_pino_peel.json",
    [string]$Ssh = "ssh",
    [string]$Scp = "scp",
    [switch]$Replay,
    [string]$Kp = "50,2:150,3:150",
    [string]$Kd = "3,2:10,3:10,4:5,5:4,6:4,7:4"
)

$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $PSScriptRoot))
Set-Location $repo

# 1. where is the arm now (read-only)
$dump = Join-Path $env:TEMP "rebot_pose.json"
& $Ssh -o BatchMode=yes $Pi "cd $PiDir && ./rebot-edge-rust dump --ids 1,2,3,4,5,6,7" |
    Set-Content -Encoding utf8 $dump
Write-Host "pose: $(Get-Content $dump -Raw | ConvertFrom-Json | Select-Object -ExpandProperty q)"

# 2. plan inside the container; its files are copied in, the JSON comes back out
docker image inspect $Image *> $null
if ($LASTEXITCODE -ne 0) {
    Write-Host "image missing: docker build -t $Image -f examples/host/Dockerfile.pinocchio ."
    exit 1
}
$id = docker create $Image python /work/plan.py --joints /work/current.json --out /work/seq.json
try {
    docker cp examples/host/app/plan_with_pinocchio.py "${id}:/work/plan.py" | Out-Null
    docker cp $dump "${id}:/work/current.json" | Out-Null
    docker start -a $id
    if ($LASTEXITCODE -ne 0) { throw "the planner failed" }
    docker cp "${id}:/work/seq.json" $Out | Out-Null
} finally {
    docker rm $id | Out-Null
}
Write-Host "wrote $Out"

# 3. optionally play it on the arm, with logging
if ($Replay) {
    & $Scp $Out "${Pi}:${PiDir}/$(Split-Path -Leaf $Out)"
    & $Ssh -o BatchMode=yes $Pi "cd $PiDir && ./rebot-edge-rust $(Split-Path -Leaf $Out) --execute --kp $Kp --kd $Kd --log repeat.json"
}
