# Fila Dry Box 

### Vídeo Apresentação:
[![apresentação do projeto](https://img.youtube.com/vi/StpMNvTfC0s/maxresdefault.jpg)](https://www.youtube.com/watch?v=StpMNvTfC0s)

### Apresentação:
O projeto proposto a seguir consiste em uma Estufa de Secagem de Filamentos para Impressão 3D, com objetivo de resolver um problema comum enfrentado por muitas pessoas que utilizam impressoras 3D: o impacto da umidade nos filamentos de impressão. Filamentos como o PLA e o ABS, quando expostos à umidade, podem ter suas propriedades prejudicadas devido a sua alta capacidade de absorção de água, resultando em falhas nas impressões e diminuição da qualidade dos objetos produzidos. Visando acabar com esse problema foi criada uma estufa automatizada capaz de controlar de forma precisa a temperatura e a umidade, proporcionando condições ideais para a secagem desses filamentos.
O sistema é composto de sensores de temperatura e umidade de alta precisão e uma interface de controle intuitiva para monitoramento e ajustes em tempo real, baseada no kit de desenvolvimento BitDog Lab. Essa estufa garante que os filamentos sejam mantidos em um ambiente livre de umidade excessiva, prolongando sua vida útil e melhorando a qualidade das peças impressas em 3D.
### Descrição do funcionamento:
A estufa de secagem de filamentos 3D foi desenvolvida utilizando o microcontrolador RP2040W presente na BitDog Lab, que é responsável por gerenciar todo o funcionamento do sistema. O RP2040W realiza o controle da temperatura e monitora a umidade interna, permitindo a interação do usuário por meio de um display OLED e dois botões físicos.
O aquecimento da estufa é feito por uma lâmpada incandescente de baixa potência, que gera calor suficiente para manter os filamentos na temperatura de secagem ideal de 45 ºC a 50 ºC para filamento do tipo PLA e de 65 ºC a 70 ºC para filamento do tipo ABS. O acionamento da lâmpada é feito por meio de um módulo relé conectado ao microcontrolador, que liga e desliga o aquecimento conforme a necessidade. 
Para o monitoramento das condições internas, é utilizado o sensor HDC1080, que mede a temperatura e a umidade relativa dentro da estufa. Esses valores são processados pelo RP2040W e exibidos no display OLED, permitindo que o usuário acompanhe o estado da secagem em tempo real.
A interação com o sistema é feita por dois botões: botão A: Liga e desliga o sistema; botão B: Alterna entre os modos de secagem: PLA e ABS, ajustando os parâmetros de temperatura conforme o tipo de filamento selecionado.
O display OLED exibe as informações essenciais, como temperatura, umidade, status do sistema e o tipo de filamento selecionado. Dessa forma, o usuário pode acompanhar o processo de secagem de forma intuitiva. 
### Justificativa:
A umidade é um dos principais problemas enfrentados na impressão 3D com filamentos termoplásticos, como PLA, ABS, PETG e Nylon. Quando expostos a umidade do ambiente, esses materiais absorvem água, o que pode resultar em diversos problemas durante a impressão, como bolhas, falhas de adesão, irregularidades na extrusão e degradação da qualidade mecânica da peça final.
Brasília, por exemplo, apresenta uma variação significativa na umidade relativa do ar ao longo do ano, podendo atingir níveis altos em certas épocas. Isso torna essencial um meio confiável de armazenamento e secagem dos filamentos para garantir a repetibilidade e qualidade das impressões.
O mercado oferece soluções como estufas comerciais e sistemas de desumidificação, mas muitas dessas alternativas são caras ou não otimizadas para o uso contínuo de filamentos. O projeto desta estufa de secagem de filamentos 3D visa oferecer uma solução eficiente, de baixo custo e acessível, permitindo o controle preciso de temperatura e umidade, garantindo que os filamentos sejam armazenados e utilizados nas melhores condições possíveis.
Além disso, a proposta busca desenvolver um sistema automatizado e personalizável, onde o usuário possa ajustar as condições de secagem conforme o tipo de filamento utilizado. Isso proporcionará maior flexibilidade e eficiência no processo de impressão 3D, reduzindo desperdícios e otimizando os resultados
