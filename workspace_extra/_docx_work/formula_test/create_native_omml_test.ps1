$ErrorActionPreference = 'Stop'
$output = 'D:\NPU\Matlab_code\output\OMML_to_MathType_minimal_test.docx'
$word = $null
$doc = $null
try {
    $word = New-Object -ComObject Word.Application
    $word.Visible = $false
    $word.DisplayAlerts = 0
    $doc = $word.Documents.Add()

    $title = $doc.Paragraphs.Add()
    $title.Range.Text = 'OMML to MathType minimal conversion test'
    $title.Range.Font.Name = 'Calibri'
    $title.Range.Font.Size = 16
    $title.Range.Font.Bold = 1
    $title.Range.InsertParagraphAfter()

    $note = $doc.Paragraphs.Add()
    $note.Range.Text = 'This file contains one equation created by the current Word native equation engine. It is not inside a table, text box, header, or comment.'
    $note.Range.Font.Name = 'Calibri'
    $note.Range.Font.Size = 10.5
    $note.Range.InsertParagraphAfter()

    $p = $doc.Paragraphs.Add()
    $p.Alignment = 1
    $range = $p.Range
    $range.Text = 'x=(-b+-sqrt(b^2-4ac))/(2a)'
    $range.End = $range.End - 1
    $null = $doc.OMaths.Add($range)
    $doc.OMaths.BuildUp()
    $p.Range.InsertParagraphAfter()

    $help = $doc.Paragraphs.Add()
    $help.Range.Text = 'Use MathType > Convert Equations. Set range to Whole document, source to Microsoft Word equations, and destination to MathType equations.'
    $help.Range.Font.Name = 'Calibri'
    $help.Range.Font.Size = 10.5

    $doc.SaveAs2($output, 16)
    [PSCustomObject]@{
        Path = $output
        OMathCount = $doc.OMaths.Count
        InTable = $doc.OMaths.Item(1).Range.Information(12)
        StoryType = $doc.OMaths.Item(1).Range.StoryType
    } | ConvertTo-Json
}
finally {
    if ($doc -ne $null) { $doc.Close($false) }
    if ($word -ne $null) { $word.Quit() }
}
