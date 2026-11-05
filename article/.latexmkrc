$pdf_mode = 1;
$out_dir = '.build';
$pdflatex = 'pdflatex -interaction=nonstopmode -halt-on-error %O %S';
$success_cmd = 'cp .build/main.pdf main.pdf';
