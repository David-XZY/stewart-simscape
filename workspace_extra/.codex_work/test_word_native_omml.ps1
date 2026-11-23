param(
    [string]$OutputDirectory = "D:\NPU\Matlab_code\output\word_formula_skill_test"
)

$ErrorActionPreference = 'Stop'
New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null

$docxPath = Join-Path $OutputDirectory 'Word_Native_OMML_Formula_Test.docx'
$pdfPath = Join-Path $OutputDirectory 'Word_Native_OMML_Formula_Test.pdf'

$word = $null
$document = $null
try {
    $word = New-Object -ComObject Word.Application
    $word.Visible = $false
    $word.DisplayAlerts = 0
    $document = $word.Documents.Add()

    $selection = $word.Selection
    $selection.Font.Name = 'Microsoft YaHei'
    $selection.Font.Size = 12
    $selection.TypeText('Word Native OfficeMath (OMML) Isolated Test')
    $selection.TypeParagraph()
    $selection.TypeText('Target formula:')
    $selection.TypeParagraph()

    # UnicodeMath is parsed and built by Word itself. No OMML-to-MathType conversion is invoked.
    $delta = [char]0x03B4
    $infinity = [char]0x221E
    $linear = "J_(LQR,k)=\\int_(0)^$infinity(${delta}x^T Q${delta}x+${delta}F^T R${delta}F)dt, ${delta}F_(LQR)=-K_k ${delta}x"
    $formulaRange = $selection.Range
    $formulaRange.Text = $linear
    $formulaRange.OMaths.Add($formulaRange) | Out-Null
    $oMath = $formulaRange.OMaths.Item(1)
    $oMath.BuildUp()

    $selection.SetRange($formulaRange.End, $formulaRange.End)
    $selection.TypeParagraph()
    $selection.TypeText('Pass condition: OMathCount=1; all tokens are native, editable Word equation structures.')

    $document.SaveAs2($docxPath, 16)
    $document.ExportAsFixedFormat($pdfPath, 17)

    [pscustomobject]@{
        DocxPath = $docxPath
        PdfPath = $pdfPath
        OMathCount = $document.OMaths.Count
        FormulaText = $oMath.Range.Text
        WordBuild = $word.Build
    } | ConvertTo-Json -Compress
}
finally {
    if ($null -ne $document) { $document.Close(0) }
    if ($null -ne $word) { $word.Quit() }
}
