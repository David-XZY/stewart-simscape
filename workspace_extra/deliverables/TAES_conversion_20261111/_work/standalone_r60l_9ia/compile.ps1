$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    for ($pass = 1; $pass -le 2; $pass++) {
        & xelatex -interaction=nonstopmode -halt-on-error -file-line-error -no-shell-escape main_taes.tex
        if ($LASTEXITCODE -ne 0) { throw "XeLaTeX failed on pass $pass" }
    }
    Write-Output 'Created main_taes.pdf'
} finally {
    Pop-Location
}
