% FIT class file
% by Masamichi HOSODA

\NeedsTeXFormat{pLaTeX2e}
\ProvidesClass{fitj}[2004/07/12 v0.43 Masamichi HOSODA]
\LoadClass[twocolumn, a4paper, papersize]{jsarticle}[2004/04/29]

\renewcommand\maketitle{%
\twocolumn[\@maketitle]
}

\def\@maketitle{%
\newpage%
\begin{center}%
    {\LARGE {\@jtitle}} \\[1zh]
    {\Large {\textsf{\@etitle}}} \\[1.5ex]
    \large{\textgt{\textsf{%
      \begin{tabular}{\@authors}
        \@jauthors \\
        \@eauthors
      \end{tabular}}}} \\
      \mbox {} \\
\end{center}
 \gdef\@jtitle{}
 \gdef\@jauthors{}
 \gdef\@etitle{}
 \gdef\@eauthors{}
 \gdef\@authors{}
}

\long\def\jtitle#1{\long\gdef\@jtitle{#1}}
\long\def\jauthors#1{\long\gdef\@jauthors{#1}}
\long\def\etitle#1{\long\gdef\@etitle{#1}}
\long\def\eauthors#1{\long\gdef\@eauthors{#1}}
\long\def\authors#1{\long\gdef\@authors{#1}}

\pagestyle{empty}

%
% �p���T�C�Y A4
% 297mm x 210mm
%

%
% ���e�̈�
% �c 242mm = �p���T�C�Y 297mm - ( ���}�[�W�� 30mm + ���}�[�W�� 25mm )
% �� 170mm = �p���T�C�Y 210mm - ( ���}�[�W�� 20mm + �E�}�[�W�� 20mm )
%          = (�J������ 81.5mm + �J�����ԃ}�[�W�� 7mm + �J������ 81.5mm)
%
\setlength\textheight{242mm}
\setlength\textwidth{170mm}

%
% ���}�[�W��
%
\setlength\topmargin{30mm}
\addtolength\topmargin{-1in}
\setlength\headheight{0mm}
\setlength\headsep{0mm}
\setlength\topskip{0mm}

%
% ���}�[�W��
%
\setlength\oddsidemargin{20mm}
\addtolength\oddsidemargin{-1in}
\setlength\evensidemargin{\oddsidemargin}

%
% �J�����ԃ}�[�W��
%
\setlength\columnsep{7mm}

%
% �s����(12pt)
%
\narrowbaselines
