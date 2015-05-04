% LaTeX Dokumentenklasse für die LNI Reihe der GI
%
% (C) 2002 Robert Tolksdorf
% Im Fall von Fragen zu dieser Dokumentenklasse wenden Sie sich bitte an mail@robert-tolksdorf.de
% Überarbeitung auf Version 0.5 im März 2010 von Judith Michael, Alpen-Adria-Universität Klagenfurt
%
%
\ProvidesClass{lni}[2010/03/01 v0.5]

%\newif\ifpdf\ifx\pdfoutput\undefined\pdffalse\else\pdfoutput=1\pdftrue\fi

% Satzspiegelposition gemäß LNI-Herausgeberrichtlinien (Korrektur von Thomas.Kuehne@mcs.vuw.ac.nz)
\setlength{\hoffset}{-0,2cm}% correct left margin to 4.2cm
\setlength{\voffset}{2.60cm}% correct top margin to 5.25cm

% Vielen Dank an Matthias Rust (mrust@rostock.zgdv.de) fuer folgende
% Ausrichtung an LNI Satzspiegel
%\DeclareOption{forInclusion}{%
%  \setlength{\topmargin}{0mm}%
%  \setlength{\headheight}{0mm}%
%  \setlength{\headsep}{0mm}%
%  \setlength{\hoffset}{-10,9mm}% 
%  \setlength{\voffset}{-3,9mm}% 
%  \setlength{\oddsidemargin}{0mm}%
%  \setlength{\evensidemargin}{0mm}%
%  \setlength{\paperheight}{235mm}%
%  \setlength{\paperwidth}{155mm}
%  \ifpdf%
%    \setlength{\pdfpageheight}{235mm}%
%    \setlength{\pdfpagewidth}{155mm}%
%  \fi}

\newif\iflnienglish\lnienglishfalse
\DeclareOption{english}{\lnienglishtrue}

\ProcessOptions

% 10 pt, A4, Seitenabstände
\LoadClass[10pt,twoside, a4paper, fleqn]{article}
\RequirePackage[english,ngerman]{babel}
\iflnienglish\selectlanguage{english}\def\bbl@main@language{english}\fi
% Blocksatz durch LaTeX-Default


% Satzspiegel
\setlength{\textheight}{19.2cm}
\setlength{\textwidth}{12.6cm}
\setlength{\topmargin}{0.17cm}%
\setlength{\oddsidemargin}{1.85cm}% korrigierte twoside werte
\setlength{\evensidemargin}{1.85cm}% korrigierte twoside werte
\setlength{\headsep}{0.31cm}%

% Ragged bottom - verhindert die Ausdehnung der Seite = Veränderung der Abstände
\def\thisbottomragged{\def\@textbottom{\vskip\z@ plus.0001fil
\global\let\@textbottom\relax}}

%Seitenzahlen - Größe der Box
\renewcommand\@pnumwidth{3em}
\renewcommand\@tocrmarg{3.5em}
\def\@dottedtocline#1#2#3#4#5{%
  \ifnum #1>\c@tocdepth \else
    \vskip \z@ \@plus.2\p@
    {\leftskip #2\relax \rightskip \@tocrmarg \advance\rightskip by 0pt plus 2cm
               \parfillskip -\rightskip \pretolerance=10000
     \parindent #2\relax\@afterindenttrue
     \interlinepenalty\@M
     \leavevmode
     \@tempdima #3\relax
     \advance\leftskip \@tempdima \null\nobreak\hskip -\leftskip
     {#4}\nobreak
     \leaders\hbox{$\m@th
        \mkern \@dotsep mu\hbox{.}\mkern \@dotsep
        mu$}\hfill
     \nobreak
     \hb@xt@\@pnumwidth{\hfil\normalfont \normalcolor #5}%
     \par}%
  \fi}
  
\frenchspacing    % Abstand nach Satzende
\widowpenalty=10000   % Vermeidet "widows & orphans"
\clubpenalty=10000


% Times als Grundschrift
\RequirePackage{mathptmx} 
%Times New Roman PS MT durch Distiller Einstellungen

% Title: Kopie aus article.cls mit anderem \thispagestyle
\renewcommand\maketitle{\par
%Keine Vertikale Einrückung über dem Titel auf der Titelseite
%\changepage{0mm}{0mm}{0mm}{0mm}{0mm}{0.56cm}{0mm}{0mm}{0mm}  
\begingroup
    \renewcommand\thefootnote{\@arabic\c@footnote}%
    \def\@makefnmark% keine Einrückung der Fußnoten eingestellt
		 {\rlap{\@textsuperscript{\normalfont\@thefnmark}}}%
		 \long\def\@makefntext##1{%
		 \@setpar{\@@par
		 \@tempdima = \hsize
		 \advance\@tempdima -1em
		 \parshape \@ne 0.15cm \@tempdima}%
		 \par\parindent 0.3cm \noindent
		 \hb@xt@\z@{\hss\@textsuperscript{\normalfont\@thefnmark}\,}##1}
    \if@twocolumn
      \ifnum \col@number=\@ne
        \@maketitle
      \else
        \twocolumn[\@maketitle]%
      \fi
    \else
      \newpage
      \global\@topnum\z@   % Prevents figures from going at top of page.
      \@maketitle
    \fi
    \thispagestyle{fancy}\@thanks   %keine Kopfzeile auf der ersten Seite
  \endgroup
  \setcounter{footnote}{0}%
  \global\let\thanks\relax
  \global\let\maketitle\relax
  \global\let\@maketitle\relax
  \global\let\@thanks\@empty
  \global\let\@author\@empty
  \global\let\@date\@empty
  \global\let\@title\@empty
  \global\let\title\relax
  \global\let\author\relax
  \global\let\date\relax
  \global\let\and\relax
}

\def\@maketitle{%
  \newpage
  \null
  \begin{center}%
  \vskip -24pt% Abstand vor dem Titel
  \flushleft %Linksbündig
  \let \footnote \thanks
    {\fontsize{14}{17}\fontseries{b}\selectfont \@title \par}%
    \vskip 19pt% Abstand nach dem Titel
    {\fontsize{10}{12}\selectfont
      \lineskip .5em%

        \@author
      \par}%
    \vskip 28pt%Abstand vor dem Abstract
  \end{center}%
  \par
  }

% Abstract
\renewenvironment*{abstract}{%
 \renewcommand{\abstractname}{Abstract}
 \fontsize{9}{10}\selectfont
    \noindent{\bfseries \abstractname:}%
}{\endquotation}  

% Keywords
\newenvironment{keywords}{\fontsize{9}{10}\selectfont
    \noindent{\bfseries Keywords:}}{}
%\def\keywordsname{{\bf Key words:}}

% Überschriften
\renewcommand{\section}{\@startsection{section}{1}{\z@}%
  {-16\p@ \@plus -4\p@ \@minus -4\p@}{5\p@ \@plus 4\p@ \@minus 4\p@}{\fontsize{12}{14}\fontseries{b}\selectfont}}
%  {19pt}{8pt}{\fontsize{12}{14}\fontseries{b}\selectfont}}
\renewcommand{\subsection}{\@startsection{subsection}{2}{\z@}%
  {-16\p@ \@plus -4\p@ \@minus -4\p@}{8\p@ \@plus 4\p@ \@minus 4\p@}{\fontsize{10}{12}\fontseries{b}\selectfont}}
%  {16pt}{4pt}{\fontsize{10}{12}\fontseries{b}\selectfont}}
% weitere nicht durch Formatvorlage geklärt

% Bildunterschriften
\setlength{\abovecaptionskip}{4pt}%Abstand vor der Bildunterschrift
\setlength{\belowcaptionskip}{0cm}%Abstand nach der Bildunterschrift
\setlength{\intextsep}{6pt}%Abstand nach der Grafik
\long\def\@makecaption#1#2{%
  \vskip\abovecaptionskip
  {\fontsize{5}{6}\selectfont
   \sbox\@tempboxa{#1: #2}%
  \ifdim \wd\@tempboxa >\hsize
    #1: #2\par
  \else
    \global \@minipagefalse
    \hb@xt@\hsize{\hfil\box\@tempboxa\hfil}%
  \fi}
  \vskip\belowcaptionskip}

%Abbildungen
\def\fps@figure{htbp}
\def\fnum@figure{\figurename\thinspace\thefigure}
\def \@floatboxreset {%
        \reset@font
        \small
        \@setnobreak
        \@setminipage
}
\setcounter{topnumber}{10}%maximale Anzahl gleitender Objekte am Seitenanfang
\setcounter{bottomnumber}{10}%maximale Anzahl gleitender Objekte am Seitenende
\renewcommand{\topfraction}{1.0}%Anteil den gleitende Objekte am Seitenanfang einnehmen dürfen
\renewcommand{\bottomfraction}{1.0}%Anteil den gleitende Objekte am Seitenende einnehmen dürfen

%Tabellen
\def\fps@table{htbp}
\def\fnum@table{\tablename~\thetable}
\renewcommand{\arraystretch}{1.1}%Abstand zu Linien in Tabelle
\renewenvironment{table}
               {\setlength\abovecaptionskip{6\p@}%
                \setlength\belowcaptionskip{0\p@}%
                \@float{table}}
               {\end@float}
\renewenvironment{table*}
               {\setlength\abovecaptionskip{6\p@}%
                \setlength\belowcaptionskip{0\p@}%
                \@dblfloat{table}}
               {\end@dblfloat}

% gleichungen mit richtiger einrückung, 0.5cm
% fleqn-Option oben
\setlength{\mathindent}{0.5cm}

% listings mit richtiger einrückung, 0.5 cm
\RequirePackage{verbatim}
\def\verbatim@processline{\hskip0.5cm\the\verbatim@line\par}



% Fussnoten mit 5cm Linie und richtiger Größe
\renewcommand\footnoterule{%
  \kern-3\p@
  \hrule\@width 5cm
  \kern2.6\p@}
  \newdimen\fnindent
  \fnindent0.18cm
\long\def\@makefntext#1{%
    \parindent \fnindent%
    \leftskip \fnindent% Einrückung vor der footnotemark
    \noindent
    \llap{\hb@xt@0.10cm{\hss\@makefnmark\ }}\ignorespaces#1}


% Absatzeinzüge und Abstände
\parindent0pt
\parskip 8pt

%Aufzählungen
%mit Punkt
\renewcommand*\itemize{%
  \ifnum \@itemdepth >\thr@@\@toodeep\else
  	\setlength{\labelsep}{0.70cm}%Abstand zum Aufzählungspunkt
    \advance\@itemdepth\@ne
    \edef\@itemitem{labelitem\romannumeral\the\@itemdepth}%   
    \expandafter
    \list  
      \csname\@itemitem\endcsname
      {\def\makelabel##1{\hss\llap{##1}}%
       \setlength{\itemsep}{8pt}%
       \setlength{\parsep}{-2pt}}%
       %\setlength{\itemindent}{0.5cm}
       
  \fi}
  
  %Nummierierte Aufzählung
  \renewcommand{\labelenumii}{\alph{enumii})}
  \renewcommand*\enumerate{%    
  \ifnum \@enumdepth >\thr@@
      \@toodeep
    \else
		\setlength{\labelsep}{0.70cm}%Abstand zur Aufzählungsnummer      
      \advance\@enumdepth \@ne
      \edef\@enumctr{enum\romannumeral\the\@enumdepth}%
    \fi
    \@ifnextchar[{\@enumlabel@{\@enumerate@}[}{\@enumerate@}}
  \def\@enumerate@{%
    \expandafter\list\csname label\@enumctr\endcsname{%
      \usecounter{\@enumctr}%
      \def\makelabel##1{\hss\llap{##1}}
		\setlength{\labelsep}{0.6cm} %Einrückung des Aufzählungszeichens     
      \setlength{\itemsep}{8pt}%
       \setlength{\parsep}{-2pt}}
       }



% bibliography
\iflanguage{english}{\bibliographystyle{lni}}{\bibliographystyle{lnig}}
%\renewcommand{\figurename}{Fig.}
\renewenvironment{thebibliography}[1]
     {%\def\refname{Literaturverzeichnis}
      \section*{\refname}%
      \bgroup\fontsize{9}{10}\selectfont
      \list{\@biblabel{\@arabic\c@enumiv}}%
           {\settowidth\labelwidth{\@biblabel{#1}}%
            \leftmargin\labelwidth
            \advance\leftmargin\labelsep
            \@openbib@code
            \usecounter{enumiv}%
            \let\p@enumiv\@empty
            \renewcommand\theenumiv{\@arabic\c@enumiv}}%
      \sloppy
      \clubpenalty4000
      \@clubpenalty \clubpenalty
      \widowpenalty4000%
      \sfcode`\.\@m}
     {\def\@noitemerr
       {\@latex@warning{Empty `thebibliography' environment}}%
      \endlist\egroup}
