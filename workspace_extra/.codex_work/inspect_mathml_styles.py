from lxml import etree

xslt = etree.XSLT(etree.parse(r"C:\Program Files\Microsoft Office\root\Office16\MML2OMML.XSL"))
source = etree.fromstring(
    b'<math xmlns="http://www.w3.org/1998/Math/MathML"><mrow><mi mathvariant="bold-italic">q</mi><mi>k</mi><mi mathvariant="bold">A</mi></mrow></math>'
)
print(str(xslt(source)))
